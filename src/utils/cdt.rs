use spade::{
    ConstrainedDelaunayTriangulation,
    Point2,
    Triangulation,
};

/*pub fn cdt_render(cdt: &ConstrainedDelaunayTriangulation<Point2<f32>>) {
    let scale: f32 = 75.0;
    let x_offset: f32 = 400.0;
    let y_offset: f32 = 300.0;

    // Edges
    for edge in cdt.undirected_edges() {
        let vertices = edge.vertices();

        let a = vertices[0].position();
        let b = vertices[1].position();

        let color = if edge.is_constraint_edge() {
            YELLOW
        } else {
            DARKGRAY
        };

        let thickness = if edge.is_constraint_edge() {
            3.0
        } else {
            1.0
        };

        draw_line(
            a.x * scale + x_offset,
            a.y * scale + y_offset,
            b.x * scale + x_offset,
            b.y * scale + y_offset,
            thickness,
            color,
        );
    }

    // Vertices
    for vertex in cdt.vertices() {
        let p = vertex.position();

        draw_circle(
            p.x * scale + x_offset,
            p.y * scale + y_offset,
            2.5,
            WHITE,
        );
    }
}*/
