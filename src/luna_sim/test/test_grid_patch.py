"""Rasterization, the accumulate-only rule, and the OccupancyGridUpdate sub-rectangle."""
from luna_sim.sensors.mock_obstacle_detector import (
    cells_bbox, extract_patch, obstacle_radius, rasterize_circle,
)

RES = 0.05
WIDTH = 10
HEIGHT = 10
ORIGIN_X = 0.0
ORIGIN_Y = 0.0
OCCUPIED = 100


def raster(cx, cy, radius):
    return rasterize_circle(cx, cy, radius, ORIGIN_X, ORIGIN_Y, RES, WIDTH, HEIGHT)


def test_obstacle_radius_halves_the_rim_diameter():
    assert obstacle_radius({"diameter": 0.4}) == 0.2
    assert obstacle_radius({"diameter": 0.1}) == 0.05


def test_tiny_circle_on_a_cell_center_marks_exactly_that_cell():
    # Cell (0, 0) center is (0.025, 0.025).
    assert set(raster(0.025, 0.025, 0.01)) == {(0, 0)}


def test_cell_is_marked_by_its_center_not_by_overlap():
    # Circle covers part of cell (0, 0) but misses its center at (0.025, 0.025).
    assert raster(0.0, 0.0, 0.02) == []


def test_filled_circle_excludes_bbox_corners():
    # r = 0.1 about (0.25, 0.25): 4x4 candidate cells, 4 corners fall outside the circle.
    cells = raster(0.25, 0.25, 0.1)
    assert len(cells) == 12
    for corner in ((3, 3), (3, 6), (6, 3), (6, 6)):
        assert corner not in cells
    assert (4, 4) in cells and (5, 5) in cells


def test_rasterization_clips_to_the_grid():
    cells = raster(0.0, 0.0, 0.1)
    # Order is not part of the contract; only the set of cells is.
    assert set(cells) == {(0, 0), (1, 0), (0, 1)}
    assert len(cells) == len(set(cells)), "no duplicate cells"
    assert all(0 <= c < WIDTH and 0 <= r < HEIGHT for c, r in cells)


def test_circle_entirely_off_the_grid_marks_nothing():
    assert raster(-5.0, -5.0, 0.2) == []
    assert raster(99.0, 99.0, 0.2) == []


def test_degenerate_inputs_mark_nothing():
    assert raster(0.25, 0.25, 0.0) == []
    assert raster(0.25, 0.25, -0.1) == []
    assert rasterize_circle(0.25, 0.25, 0.1, ORIGIN_X, ORIGIN_Y, 0.0, WIDTH, HEIGHT) == []


def test_bbox_of_scattered_cells():
    assert cells_bbox([(3, 5), (4, 5), (10, 7)]) == (3, 5, 8, 3)


def test_bbox_of_a_single_cell_is_one_by_one():
    assert cells_bbox([(2, 7)]) == (2, 7, 1, 1)


def test_bbox_of_nothing_is_none():
    assert cells_bbox([]) is None


def test_bbox_covers_every_rasterized_cell():
    cells = raster(0.25, 0.25, 0.1)
    x0, y0, w, h = cells_bbox(cells)
    assert all(x0 <= c < x0 + w and y0 <= r < y0 + h for c, r in cells)


def test_extract_patch_is_row_major_and_offset_correctly():
    grid = list(range(12))        # 4 wide, 3 tall
    assert extract_patch(grid, 4, 1, 1, 2, 2) == [5, 6, 9, 10]


def test_extract_patch_of_the_whole_grid_is_the_grid():
    grid = list(range(12))
    assert extract_patch(grid, 4, 0, 0, 4, 3) == grid


def test_patch_carries_the_occupied_value_and_the_walls_underneath():
    grid = [0] * (WIDTH * HEIGHT)
    grid[0] = OCCUPIED           # a wall cell from /map, inside the patch region

    for col, row in raster(0.1, 0.1, 0.06):
        grid[row * WIDTH + col] = OCCUPIED

    x0, y0, w, h = cells_bbox(raster(0.1, 0.1, 0.06))
    data = extract_patch(grid, WIDTH, x0, y0, w, h)
    assert len(data) == w * h
    assert set(data) <= {0, OCCUPIED}
    assert OCCUPIED in data


def test_accumulate_only_reports_nothing_new_the_second_time():
    grid = [0] * (WIDTH * HEIGHT)

    def mark(cells):
        new = []
        for col, row in cells:
            i = row * WIDTH + col
            if grid[i] != OCCUPIED:
                grid[i] = OCCUPIED
                new.append((col, row))
        return new

    cells = raster(0.25, 0.25, 0.1)
    assert len(mark(cells)) == len(cells)      # first detection: all cells are new
    assert mark(cells) == []                   # same detection again: nothing to publish
    assert cells_bbox(mark(cells)) is None


def test_a_shifted_detection_only_reports_the_newly_covered_cells():
    grid = [0] * (WIDTH * HEIGHT)

    def mark(cells):
        new = []
        for col, row in cells:
            i = row * WIDTH + col
            if grid[i] != OCCUPIED:
                grid[i] = OCCUPIED
                new.append((col, row))
        return new

    first = raster(0.25, 0.25, 0.1)
    mark(first)

    # Same obstacle, noise nudged it one cell over: the overlap must not be re-reported.
    second = raster(0.30, 0.25, 0.1)
    new = mark(second)

    assert new, "a shifted detection should cover at least one fresh cell"
    assert set(new) <= set(second)
    assert not set(new) & set(first)

    # The patch bbox covers only the new cells, not the whole union.
    x0, y0, w, h = cells_bbox(new)
    assert all(x0 <= c < x0 + w and y0 <= r < y0 + h for c, r in new)
    assert w <= WIDTH and h <= HEIGHT


def test_patch_data_length_always_matches_the_bbox():
    grid = [0] * (WIDTH * HEIGHT)
    for col, row in raster(0.25, 0.25, 0.12):
        grid[row * WIDTH + col] = OCCUPIED

    x0, y0, w, h = cells_bbox(raster(0.25, 0.25, 0.12))
    assert len(extract_patch(grid, WIDTH, x0, y0, w, h)) == w * h
