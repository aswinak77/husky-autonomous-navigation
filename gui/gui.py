import cv2
import numpy as np


class GUI:
    GRID_SIZE  = 30
    CELL_SIZE  = 20

    WORLD_SCALE = 0.5

    C_BACKGROUND = (245, 245, 245)
    C_GRID       = (200, 200, 200)
    C_OBSTACLE   = (40,  40,  40)
    C_START      = (50,  205, 50)
    C_GOAL       = (30,  30,  220)
    C_PATH       = (200, 160, 0)
    C_TEXT       = (50,  50,  50)

    def __init__(self, grid_size: int = GRID_SIZE, cell_size: int = CELL_SIZE):
        self.grid_size  = grid_size
        self.cell_size  = cell_size
        self.px         = grid_size * cell_size

        self.start = None
        self.goal  = None
        self.path  = []

        self.grid = np.zeros((grid_size, grid_size), dtype=np.uint8)
        self._add_default_obstacles()
        self.grid = self._inflate_obstacles(self.grid, padding=2)

        self.window_name = "Husky Navigator - Grid GUI"

        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)

        dummy = np.zeros((self.px, self.px, 3), dtype=np.uint8)
        cv2.imshow(self.window_name, dummy)

        cv2.setMouseCallback(self.window_name, self._click_event)


    def get_world_scale(self) -> float:
        return self.WORLD_SCALE

    def draw_path(self, path: list):
        self.path = path

    def run(self):
        while True:
            frame = self._render()
            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(16) & 0xFF

            # detect window close
            if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1:
                print("[GUI] Window closed by user")
                cv2.destroyAllWindows()
                return None, None, None

            # ESC to exit
            if key == 27:
                cv2.destroyAllWindows()
                return None, None, self.grid

            if key == ord('r'):
                self.start = None
                self.goal  = None
                self.path  = []
                print("Selection reset.")

            if self.start is not None and self.goal is not None:
                for _ in range(30):
                    cv2.imshow(self.window_name, self._render())
                    cv2.waitKey(16)

                cv2.destroyAllWindows()
                return self.start, self.goal, self.grid



    def _add_default_obstacles(self):
        g = self.grid
        g[5:8,  5:8]  = 1
        g[10:15, 10] = 1
        g[15:20, 15] = 1
        g[3:6,   20] = 1
        g[20:25, 8]  = 1

    @staticmethod
    def _inflate_obstacles(grid: np.ndarray, padding: int = 1) -> np.ndarray:
        inflated = grid.copy()
        rows, cols = grid.shape

        for i in range(rows):
            for j in range(cols):
                if grid[i, j] == 1:
                    for di in range(-padding, padding + 1):
                        for dj in range(-padding, padding + 1):
                            ni, nj = i + di, j + dj
                            if 0 <= ni < rows and 0 <= nj < cols:
                                inflated[ni, nj] = 1

        return inflated

    def _render(self) -> np.ndarray:
        cs = self.cell_size
        img = np.full((self.px, self.px, 3), self.C_BACKGROUND, dtype=np.uint8)

        for i in range(self.grid_size + 1):
            cv2.line(img, (0, i*cs), (self.px, i*cs), self.C_GRID, 1)
            cv2.line(img, (i*cs, 0), (i*cs, self.px), self.C_GRID, 1)

        for r in range(self.grid_size):
            for c in range(self.grid_size):
                if self.grid[r, c] == 1:
                    cv2.rectangle(img,
                                  (c*cs, r*cs),
                                  ((c+1)*cs, (r+1)*cs),
                                  self.C_OBSTACLE, -1)

        for (r, c) in self.path:
            cx = c*cs + cs//2
            cy = r*cs + cs//2
            cv2.circle(img, (cx, cy), max(2, cs//5), self.C_PATH, -1)

        if self.start:
            r, c = self.start
            cv2.rectangle(img, (c*cs, r*cs), ((c+1)*cs, (r+1)*cs), self.C_START, -1)
            cv2.putText(img, "S", (c*cs+3, (r+1)*cs-3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,255), 1)

        if self.goal:
            r, c = self.goal
            cv2.rectangle(img, (c*cs, r*cs), ((c+1)*cs, (r+1)*cs), self.C_GOAL, -1)
            cv2.putText(img, "G", (c*cs+3, (r+1)*cs-3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,255), 1)

        if self.start is None:
            status = "Left click to set START"
        elif self.goal is None:
            status = "Left click to set GOAL | Press R to reset"
        else:
            status = "Starting simulation."

        cv2.rectangle(img, (0, self.px - 22), (self.px, self.px), (230,230,230), -1)
        cv2.putText(img, status, (6, self.px - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, self.C_TEXT, 1)

        return img

    def _click_event(self, event, x, y, flags, param):
        r = y // self.cell_size
        c = x // self.cell_size

        if not (0 <= r < self.grid_size and 0 <= c < self.grid_size):
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            if self.grid[r, c] == 1:
                print("That cell is an obstacle — choose another")
                return

            if self.start is None:
                self.start = (r, c)
                print(f"Start set: row={r}, col={c}")
            elif self.goal is None:
                self.goal = (r, c)
                print(f"Goal set: row={r}, col={c}")