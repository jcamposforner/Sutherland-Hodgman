use std::error::Error;

#[derive(Debug, Copy, Clone)]
struct Point {
    x: f64,
    y: f64,
}

impl Eq for Point {}

impl PartialEq for Point {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y
    }
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

    fn is_inside(&self, point: &Point) -> bool {
        self.cross_product(point) >= 0.0
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
            Some(Point::new(x, y))
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
}

#[derive(Debug)]
enum PointPosition {
    Inside(Point),
    Outside(Point),
}

impl PointPosition {
    fn is_inside(point: Point, line: &Line) -> Self {
        if line.is_inside(&point) {
            PointPosition::Inside(point)
        } else {
            PointPosition::Outside(point)
        }
    }
}

struct PointPositions {
    start: PointPosition,
    end: PointPosition,
}

impl PointPositions {
    fn new(start: PointPosition, end: PointPosition) -> Self {
        PointPositions { start, end }
    }

    fn calculate_vertexes(self, line: &Line) -> Vec<Point> {
        let mut vertexes = vec![];
        match (self.start, self.end) {
            (PointPosition::Inside(_), PointPosition::Inside(end)) => {
                vertexes.push(end);
            }
            (PointPosition::Inside(start), PointPosition::Outside(end)) => {
                Self::calculate_intersection(
                    line,
                    &Line::new(start, end),
                    &mut vertexes,
                );
            }
            (PointPosition::Outside(start), PointPosition::Inside(end)) => {
                let intersection_line = Line::new(start, end);
                Self::calculate_intersection(
                    line,
                    &intersection_line,
                    &mut vertexes,
                );

                vertexes.push(intersection_line.end);
            }
            _ => {}
        }

        vertexes
    }

    fn calculate_intersection(
        line: &Line,
        intersection_line: &Line,
        vertexes: &mut Vec<Point>,
    ) {
        let intersection = line.intersection(&intersection_line);
        if let Some(intersection) = intersection {
            vertexes.push(intersection);
        }
    }
}

struct SutherlandHodgman;

impl SutherlandHodgman {
    fn remove_duplicated_points(output_polygon: Vec<Point>) -> Vec<Point> {
        let mut unique_points: Vec<Point> = Vec::new();
        for point in output_polygon {
            if unique_points.last() != Some(&point) {
                unique_points.push(point);
            }
        }

        unique_points
    }
}

impl ClippingStrategy for SutherlandHodgman {
    fn clip_polygon(&self, clipping_polygon: &Polygon, input_polygon: &Polygon) -> Option<Polygon> {
        let mut output_polygon = input_polygon.vertexes.clone();

        for i in 0..clipping_polygon.vertexes.len() {
            let mut new_vertexes = vec![];
            let clipping_start = &clipping_polygon.vertexes[i];
            let clipping_end = &clipping_polygon.vertexes[(i + 1) % clipping_polygon.vertexes.len()];
            let clipping_line = Line::new(*clipping_start, *clipping_end);

            for j in 0..output_polygon.len() {
                let start_point = &output_polygon[j];
                let end_point = &output_polygon[(j + 1) % output_polygon.len()];

                let current_position = PointPosition::is_inside(*start_point, &clipping_line);
                let next_position = PointPosition::is_inside(*end_point, &clipping_line);

                PointPositions::new(current_position, next_position)
                    .calculate_vertexes(&clipping_line)
                    .into_iter()
                    .for_each(|vertex| new_vertexes.push(vertex));
            }

            output_polygon = new_vertexes;
        }

        if output_polygon.is_empty() {
            return None;
        }

        let unique_points = Self::remove_duplicated_points(output_polygon);

        Some(Polygon::new(unique_points))
    }
}

trait ClippingStrategy {
    fn clip_polygon(&self, clipping_polygon: &Polygon, input_polygon: &Polygon) -> Option<Polygon>;
}

struct PolygonClippingCalculator<'a, C: ClippingStrategy> {
    strategy: &'a C,
}

impl<'a, C: ClippingStrategy> PolygonClippingCalculator<'a, C> {
    fn new(strategy: &'a C) -> Self {
        Self { strategy }
    }

    fn clip_polygon(&self, polygon: &Polygon, clipping_polygon: &Polygon) -> Option<Polygon> {
        self.strategy.clip_polygon(polygon, clipping_polygon)
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

    let hodgman = SutherlandHodgman;
    let calculator = PolygonClippingCalculator::new(&hodgman);
    let result = calculator.clip_polygon(&square, &clipping_polygon);
    println!("{:#?}", result);

    Ok(())
}
