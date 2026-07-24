use std::sync::mpsc;
use eframe::egui::{self, Color32, Pos2, Rect, Stroke, Vec2, Ui};

use crate::navigation::Point;

#[derive(Debug, Clone)]
pub struct StaticGeometry {
    pub navmesh_polygons: Vec<Vec<Point>>,
    pub navgraph_edges: Vec<(Point, Point)>,
}

#[derive(Debug, Clone, Default)]
pub struct DynamicState {
    pub position: Option<Point>,
    pub heading: Option<Point>,
    pub path: Option<Vec<Point>>,
}

#[derive(Debug, Clone)]
pub enum RenderUpdate {
    Static(StaticGeometry),
    Dynamic(DynamicState),
}

pub fn render(
    render_rx: mpsc::Receiver<RenderUpdate>,
    click_tx: mpsc::Sender<Point>,
) -> anyhow::Result<()> {
    let native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "SINA - Navigator",
        native_options,
        Box::new(move |_cc| Ok(Box::new(RenderApp::new(render_rx, click_tx)))),
    ).map_err(|e| anyhow::anyhow!("eframe failed: {e}"))?;
    Ok(())
}

struct ViewTransform {
    scale: f32,
    offset: Vec2,
}

impl ViewTransform {
    fn identity() -> Self {
        Self { scale: 1.0, offset: Vec2::ZERO }
    }

    fn fit_to_geometry(geo: &StaticGeometry, panel: Rect) -> Self {
        let mut min = Pos2::new(f32::INFINITY, f32::INFINITY);
        let mut max = Pos2::new(f32::NEG_INFINITY, f32::NEG_INFINITY);

        let mut extend = |p: Point| {
            let x = p.x.into_inner();
            let y = p.y.into_inner();
            min.x = min.x.min(x); min.y = min.y.min(y);
            max.x = max.x.max(x); max.y = max.y.max(y);
        };

        for poly in &geo.navmesh_polygons {
            for &p in poly { extend(p); }
        }
        for (a, b) in &geo.navgraph_edges {
            extend(*a); extend(*b);
        }

        if !min.x.is_finite() || !max.x.is_finite() {
            return Self::identity();
        }

        let world_size = (max - min).max(Vec2::splat(0.1));
        let margin = 40.0;
        let avail = (panel.size() - Vec2::splat(margin * 2.0)).max(Vec2::splat(1.0));

        let scale = (avail.x / world_size.x).min(avail.y / world_size.y);

        let world_center = min + (max - min) * 0.5;
        let offset = panel.center().to_vec2() - Vec2::new(world_center.x, -world_center.y) * scale;

        Self { scale, offset }
    }

    fn world_to_screen(&self, p: Point) -> Pos2 {
        let x = p.x.into_inner();
        let y = p.y.into_inner();
        Pos2::new(x, -y) * self.scale + self.offset
    }

    fn screen_to_world(&self, screen: Pos2) -> Point {
        let v = (screen - self.offset) / self.scale;
        (v.x as f64, -v.y as f64).into()
    }
}

struct RenderApp {
    render_rx: mpsc::Receiver<RenderUpdate>,
    click_tx: mpsc::Sender<Point>,
    static_geo: Option<StaticGeometry>,
    dynamic: DynamicState,
    view: ViewTransform,
    view_fitted: bool,
}

impl RenderApp {
    fn new(render_rx: mpsc::Receiver<RenderUpdate>, click_tx: mpsc::Sender<Point>) -> Self {
        Self {
            render_rx,
            click_tx,
            static_geo: None,
            dynamic: DynamicState::default(),
            view: ViewTransform::identity(),
            view_fitted: false,
        }
    }

    fn drain_updates(&mut self) {
        while let Ok(update) = self.render_rx.try_recv() {
            match update {
                RenderUpdate::Static(geo) => {
                    self.static_geo = Some(geo);
                    self.view_fitted = false;
                }
                RenderUpdate::Dynamic(state) => {
                    self.dynamic = state;
                }
            }
        }
    }

    fn handle_pan_zoom(&mut self, response: &egui::Response, ui: &egui::Ui) {
        if response.dragged() {
            self.view.offset += response.drag_delta();
        }
        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll != 0.0 {
            let zoom_factor = (1.0 + scroll * 0.001).clamp(0.8, 1.2);
            self.view.scale *= zoom_factor;
        }
    }

    fn handle_click(&mut self, response: &egui::Response) {
        if response.clicked() {
            if let Some(screen_pos) = response.interact_pointer_pos() {
                let world_point = self.view.screen_to_world(screen_pos);
                let _ = self.click_tx.send(world_point);
            }
        }
    }
}

impl eframe::App for RenderApp {
    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        self.drain_updates();

        egui::CentralPanel::default().show_inside(ui, |ui| {
            let (response, painter) =
                ui.allocate_painter(ui.available_size(), egui::Sense::click_and_drag());

            if let Some(geo) = &self.static_geo {
                if !self.view_fitted {
                    self.view = ViewTransform::fit_to_geometry(geo, response.rect);
                    self.view_fitted = true;
                }
                draw_navmesh(&painter, &self.view, geo);
            }

            if let Some(path) = &self.dynamic.path {
                draw_path(&painter, &self.view, path);
            }

            draw_position_heading(&painter, &self.view, &self.dynamic);

            self.handle_pan_zoom(&response, ui);
            self.handle_click(&response);
        });

        ui.ctx().request_repaint();
    }
}

fn draw_navmesh(painter: &egui::Painter, view: &ViewTransform, geo: &StaticGeometry) {
    let poly_stroke = Stroke::new(1.0, Color32::from_rgb(80, 120, 255));
    for poly in &geo.navmesh_polygons {
        if poly.len() < 2 { continue; }
        let screen_pts: Vec<Pos2> = poly.iter().map(|&p| view.world_to_screen(p)).collect();
        for i in 0..screen_pts.len() {
            let a = screen_pts[i];
            let b = screen_pts[(i + 1) % screen_pts.len()];
            painter.line_segment([a, b], poly_stroke);
        }
    }

    let edge_stroke = Stroke::new(1.0, Color32::from_rgb(150, 150, 150));
    for (a, b) in &geo.navgraph_edges {
        painter.line_segment(
            [view.world_to_screen(*a), view.world_to_screen(*b)],
            edge_stroke,
        );
    }
}

fn draw_path(painter: &egui::Painter, view: &ViewTransform, path: &[Point]) {
    if path.len() < 2 { return; }
    let stroke = Stroke::new(3.0, Color32::from_rgb(255, 165, 0));
    let screen_pts: Vec<Pos2> = path.iter().map(|&p| view.world_to_screen(p)).collect();
    for w in screen_pts.windows(2) {
        painter.line_segment([w[0], w[1]], stroke);
    }
}

fn draw_position_heading(painter: &egui::Painter, view: &ViewTransform, dynamic: &DynamicState) {
    let Some(pos) = dynamic.position else { return };
    let screen_pos = view.world_to_screen(pos);

    painter.circle_filled(screen_pos, 6.0, Color32::from_rgb(255, 0, 0));

    if let Some(heading) = dynamic.heading {
        let tip = view.world_to_screen(Point {
            x: pos.x + heading.x * ordered_float::OrderedFloat(0.6),
            y: pos.y + heading.y * ordered_float::OrderedFloat(0.6),
        });
        painter.arrow(
            screen_pos,
            tip - screen_pos,
            Stroke::new(2.0, Color32::from_rgb(0, 200, 0)),
        );
    }
}
