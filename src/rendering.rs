use std::sync::mpsc;
use eframe::egui::{self, Color32, Pos2, Rect, Stroke, Vec2, Ui};

use crate::navigation::Point;

#[derive(Debug, Clone)]
pub struct StaticGeometry {
    pub walls: Vec<(Point, Point)>,
    pub doors: Vec<(Point, Point)>,
    pub bboxes: Vec<Vec<Point>>,
    pub borders: Vec<Vec<Point>>,
    pub holes: Vec<Vec<Point>>,
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

struct LayerVisibility {
    data: bool,
    topology: bool,
    navmesh: bool,
    navgraph: bool,
}

impl LayerVisibility {
    fn default_for_build() -> Self {
        Self {
            data: true,
            topology: cfg!(debug_assertions),
            navmesh: cfg!(debug_assertions),
            navgraph: cfg!(debug_assertions),
        }
    }
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

        for (a, b) in &geo.walls { extend(*a); extend(*b); }
        for (a, b) in &geo.doors { extend(*a); extend(*b); }
        for bbox in &geo.bboxes { for &p in bbox { extend(p); } }
        for poly in &geo.borders { for &p in poly { extend(p); } }
        for poly in &geo.holes { for &p in poly { extend(p); } }
        for poly in &geo.navmesh_polygons { for &p in poly { extend(p); } }
        for (a, b) in &geo.navgraph_edges { extend(*a); extend(*b); }

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
    layers: LayerVisibility,
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
            layers: LayerVisibility::default_for_build(),
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
        // Drag = pan libre horizontal + vertical.
        if response.dragged() {
            self.view.offset += response.drag_delta();
        }

        let ctrl_held = ui.input(|i| i.modifiers.ctrl || i.modifiers.command);

        if ctrl_held {
            // Ctrl/Cmd + molette : egui convertit ça en zoom_delta (pas scroll_delta).
            let zoom_delta = ui.input(|i| i.zoom_delta());

            if zoom_delta != 1.0 {
                if let Some(cursor_pos) = response.hover_pos() {
                    let world_before = self.view.screen_to_world(cursor_pos);
                    self.view.scale *= zoom_delta;
                    let screen_after = self.view.world_to_screen(world_before);
                    self.view.offset += cursor_pos - screen_after;
                } else {
                    self.view.scale *= zoom_delta;
                }
            }
        } else {
            // Scroll simple = pan horizontal + vertical.
            let scroll_delta = ui.input(|i| i.smooth_scroll_delta);
            self.view.offset += scroll_delta;
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

    fn draw_layers_menu(&mut self, ui: &mut Ui) {
        egui::Window::new("Menu")
            .default_pos(Pos2::new(12.0, 12.0))
            .resizable(false)
            .collapsible(true)
            .show(ui.ctx(), |ui| {
                ui.checkbox(&mut self.layers.data, "Data (Walls / Doors / Bboxes)");
                ui.checkbox(&mut self.layers.topology, "Topology (borders / holes)");
                ui.checkbox(&mut self.layers.navmesh, "Navmesh");
                ui.checkbox(&mut self.layers.navgraph, "Navgraph (edges)");

                ui.separator();
                if ui.button("Recenter view").clicked() {
                    self.view_fitted = false;
                }
            });
    }
}

impl eframe::App for RenderApp {
    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        self.drain_updates();

        self.draw_layers_menu(ui);

        egui::CentralPanel::default().show_inside(ui, |ui| {
            let (response, painter) =
                ui.allocate_painter(ui.available_size(), egui::Sense::click_and_drag());

            if let Some(geo) = &self.static_geo {
                if !self.view_fitted {
                    self.view = ViewTransform::fit_to_geometry(geo, response.rect);
                    self.view_fitted = true;
                }

                if self.layers.data {
                    draw_data(&painter, &self.view, geo);
                }
                if self.layers.topology {
                    draw_topology(&painter, &self.view, geo);
                }
                if self.layers.navmesh {
                    draw_navmesh(&painter, &self.view, geo);
                }
                if self.layers.navgraph {
                    draw_navgraph_edges(&painter, &self.view, geo);
                }
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

fn draw_polygon_outline(painter: &egui::Painter, view: &ViewTransform, poly: &[Point], stroke: Stroke) {
    if poly.len() < 2 { return; }
    let screen_pts: Vec<Pos2> = poly.iter().map(|&p| view.world_to_screen(p)).collect();
    for i in 0..screen_pts.len() {
        let a = screen_pts[i];
        let b = screen_pts[(i + 1) % screen_pts.len()];
        painter.line_segment([a, b], stroke);
    }
}

fn draw_data(painter: &egui::Painter, view: &ViewTransform, geo: &StaticGeometry) {
    let wall_stroke = Stroke::new(2.0, Color32::from_rgb(80, 120, 255));
    for (a, b) in &geo.walls {
        painter.line_segment([view.world_to_screen(*a), view.world_to_screen(*b)], wall_stroke);
    }

    let door_stroke = Stroke::new(2.0, Color32::from_rgb(0, 200, 0));
    for (a, b) in &geo.doors {
        painter.line_segment([view.world_to_screen(*a), view.world_to_screen(*b)], door_stroke);
    }

    let bbox_stroke = Stroke::new(1.5, Color32::from_rgb(255, 80, 80));
    for bbox in &geo.bboxes {
        draw_polygon_outline(painter, view, bbox, bbox_stroke);
    }
}

fn draw_topology(painter: &egui::Painter, view: &ViewTransform, geo: &StaticGeometry) {
    let border_stroke = Stroke::new(1.5, Color32::from_rgb(0, 200, 255));
    for border in &geo.borders {
        draw_polygon_outline(painter, view, border, border_stroke);
    }

    let hole_stroke = Stroke::new(1.5, Color32::from_rgb(255, 100, 100));
    for hole in &geo.holes {
        draw_polygon_outline(painter, view, hole, hole_stroke);
    }
}

fn draw_navmesh(painter: &egui::Painter, view: &ViewTransform, geo: &StaticGeometry) {
    let poly_stroke = Stroke::new(1.0, Color32::from_rgb(80, 120, 255));
    for poly in &geo.navmesh_polygons {
        draw_polygon_outline(painter, view, poly, poly_stroke);
    }
}

fn draw_navgraph_edges(painter: &egui::Painter, view: &ViewTransform, geo: &StaticGeometry) {
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
