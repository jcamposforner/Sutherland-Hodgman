use std::error::Error;

#[derive(Debug, Clone)]
struct Point {
    x: f64,
    y: f64,
}

impl Point {
    fn new(x: f64, y: f64) -> Self {
        Point { x, y }
    }
}

#[derive(Debug)]
struct Line {
    start: Point,
    end: Point,
}

impl Line {
    fn new(start: Point, end: Point) -> Self {
        Line { start, end }
    }

    fn cross_product(&self, point: &Point) -> f64 {
        let vector_edge = Point::new(self.end.x - self.start.x, self.end.y - self.start.y);
        let vector_vertex = Point::new(point.x - self.start.x, point.y - self.start.y);

        vector_edge.x * vector_vertex.y - vector_edge.y * vector_vertex.x
    }

    fn is_left(&self, point: &Point) -> bool {
        self.cross_product(point) >= 0.0
    }

    fn is_inside(&self, point: &Point) -> bool {
        (self.end.x - self.start.x) * (point.y - self.start.y) >= (self.end.y - self.start.y) * (point.x - self.start.x)
    }

    fn intersection(&self, other: &Line) -> Option<Point> {
        let a1 = self.end.y - self.start.y;
        let b1 = self.start.x - self.end.x;
        let c1 = a1 * self.start.x + b1 * self.start.y;

        let a2 = other.end.y - other.start.y;
        let b2 = other.start.x - other.end.x;
        let c2 = a2 * other.start.x + b2 * other.start.y;
        let determinant = a1 * b2 - a2 * b1;

        // PARALLEL LINES
        if determinant == 0.0 {
            return None;
        }

        let x = (b2 * c1 - b1 * c2) / determinant;
        let y = (a1 * c2 - a2 * c1) / determinant;

        return if x >= f64::min(self.start.x, self.end.x) && x <= f64::max(self.start.x, self.end.x) &&
            y >= f64::min(self.start.y, self.end.y) && y <= f64::max(self.start.y, self.end.y) &&
            x >= f64::min(other.start.x, other.end.x) && x <= f64::max(other.start.x, other.end.x) &&
            y >= f64::min(other.start.y, other.end.y) && y <= f64::max(other.start.y, other.end.y) {
            Some(Point { x, y })
        } else {
            None
        };
    }
}

#[derive(Debug)]
struct Polygon {
    vertexes: Vec<Point>,
}

impl Polygon {
    fn new(vertexes: Vec<Point>) -> Self {
        Polygon { vertexes }
    }

    fn clip_polygon(&self, input_polygon: &Polygon) -> Polygon {
        let mut output_polygon = input_polygon.vertexes.clone();

        for i in 0..self.vertexes.len() {
            let mut new_vertexes = vec![];

            let edge_start = &self.vertexes[i];
            let edge_end = &self.vertexes[(i + 1) % self.vertexes.len()];
            let edge = Line::new(edge_start.clone(), edge_end.clone());

            for j in 0..output_polygon.len() {
                let current_vertex = &output_polygon[j];
                let next_vertex = &output_polygon[(j + 1) % output_polygon.len()];
                let current_is_inside = edge.is_inside(current_vertex);
                let next_is_inside = edge.is_inside(next_vertex);
                if !current_is_inside && !next_is_inside {
                    continue;
                }

                if current_is_inside && next_is_inside {
                    new_vertexes.push(next_vertex.clone());
                    continue;
                }

                let intersection_line = Line::new(current_vertex.clone(), next_vertex.clone());
                let intersection = edge.intersection(&intersection_line);

                if current_is_inside {
                    if let Some(intersection) = intersection {
                        new_vertexes.push(intersection);
                    }
                    continue;
                }

                if let Some(intersection) = intersection {
                    new_vertexes.push(intersection);
                }
                new_vertexes.push(next_vertex.clone());
            }

            output_polygon = new_vertexes.clone();
        }

        Polygon::new(output_polygon)
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let square = Polygon::new(
        vec![
            Point::new(150.0, 150.0),
            Point::new(200.0, 150.0),
            Point::new(200.0, 200.0),
            Point::new(150.0, 200.0),
        ]
    );

    let clipping_polygon = Polygon::new(
        vec![
            Point::new(100.0, 150.0),
            Point::new(200.0, 250.0),
            Point::new(300.0, 200.0),
        ]
    );

    let result = square.clip_polygon(&clipping_polygon);

    let square = Polygon::new(
        vec![
            Point::new(200.0, 100.0),
            Point::new(300.0, 300.0),
            Point::new(100.0, 300.0),
        ]
    );

    let clipping_polygon = Polygon::new(
        vec![
            Point::new(100.0, 150.0),
            Point::new(200.0, 250.0),
            Point::new(300.0, 200.0),
        ]
    );

    let result = square.clip_polygon(&clipping_polygon);

    Ok(())
}
