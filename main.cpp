// Maze Generator + Solver — single-file SDL2 implementation in C++
//
// Generation: randomized depth-first search (recursive backtracker), animated
// carve-by-carve so you can watch the maze being built.
// Solving:    breadth-first search from top-left to bottom-right, animated
// frontier-by-frontier, then the shortest path is traced in a highlight color.
//
// Controls:
//   Space  = generate a new random maze
//   Enter  = solve the current maze (BFS)
//   +/-    = increase / decrease maze size (regenerates)
//   F      = toggle fast-forward (instant generate/solve instead of animated)
//   Esc    = quit

#include <SDL2/SDL.h>
#include <array>
#include <vector>
#include <stack>
#include <queue>
#include <random>
#include <algorithm>
#include <cstdio>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
constexpr int MARGIN = 20;
constexpr int MIN_COLS = 5, MAX_COLS = 80;
constexpr int WINDOW_W = 900;
constexpr int WINDOW_H = 900;

// walls: bit flags per cell
enum Wall { WALL_N = 1, WALL_E = 2, WALL_S = 4, WALL_W = 8 };

struct Cell {
    int walls = WALL_N | WALL_E | WALL_S | WALL_W; // all walls up initially
    bool visited = false;   // used during generation
    bool bfsVisited = false; // used during solving
    int parent = -1;         // for path reconstruction after BFS
};

// ---------------------------------------------------------------------------
// Maze
// ---------------------------------------------------------------------------
class Maze {
public:
    Maze(int cols, int rows) : cols(cols), rows(rows), rng(std::random_device{}()) {
        reset();
    }

    void reset() {
        cells.assign(cols * rows, Cell{});
        genStack = std::stack<int>();
        genDone = false;
        genCurrent = 0;
        cells[0].visited = true;
        genStack.push(0);

        solving = false;
        solveDone = false;
        bfsQueue = std::queue<int>();
        path.clear();
        pathIdx = 0;
    }

    int idx(int c, int r) const { return r * cols + c; }
    int colOf(int i) const { return i % cols; }
    int rowOf(int i) const { return i / cols; }

    // returns -1 if no neighbor in that direction / out of bounds
    int neighbor(int i, Wall dir) const {
        int c = colOf(i), r = rowOf(i);
        switch (dir) {
            case WALL_N: return (r > 0) ? idx(c, r - 1) : -1;
            case WALL_S: return (r < rows - 1) ? idx(c, r + 1) : -1;
            case WALL_E: return (c < cols - 1) ? idx(c + 1, r) : -1;
            case WALL_W: return (c > 0) ? idx(c - 1, r) : -1;
        }
        return -1;
    }

    static Wall opposite(Wall w) {
        switch (w) {
            case WALL_N: return WALL_S;
            case WALL_S: return WALL_N;
            case WALL_E: return WALL_W;
            case WALL_W: return WALL_E;
        }
        return WALL_N;
    }

    // ---- generation: one carve step of randomized DFS ----
    void generateStep() {
        if (genDone || genStack.empty()) { genDone = true; return; }
        int current = genStack.top();
        genCurrent = current;

        std::array<Wall,4> dirs{WALL_N, WALL_E, WALL_S, WALL_W};
        std::shuffle(dirs.begin(), dirs.end(), rng);

        for (Wall d : dirs) {
            int n = neighbor(current, d);
            if (n != -1 && !cells[n].visited) {
                cells[current].walls &= ~d;
                cells[n].walls &= ~opposite(d);
                cells[n].visited = true;
                genStack.push(n);
                return;
            }
        }
        // no unvisited neighbor: backtrack
        genStack.pop();
        if (genStack.empty()) genDone = true;
    }

    void generateInstant() {
        while (!genDone) generateStep();
    }

    // ---- solving: BFS from top-left to bottom-right ----
    void startSolve() {
        for (auto& c : cells) { c.bfsVisited = false; c.parent = -1; }
        bfsQueue = std::queue<int>();
        int start = idx(0, 0);
        cells[start].bfsVisited = true;
        bfsQueue.push(start);
        solving = true;
        solveDone = false;
        path.clear();
        pathIdx = 0;
    }

    void solveStep() {
        if (!solving || solveDone) return;
        if (bfsQueue.empty()) { solveDone = true; solving = false; return; }
        int current = bfsQueue.front();
        bfsQueue.pop();
        int goal = idx(cols - 1, rows - 1);
        if (current == goal) {
            buildPath(goal);
            solveDone = true;
            solving = false;
            return;
        }
        static const Wall dirs[4] = {WALL_N, WALL_E, WALL_S, WALL_W};
        for (Wall d : dirs) {
            if (cells[current].walls & d) continue; // wall blocks movement
            int n = neighbor(current, d);
            if (n != -1 && !cells[n].bfsVisited) {
                cells[n].bfsVisited = true;
                cells[n].parent = current;
                bfsQueue.push(n);
            }
        }
    }

    void solveInstant() {
        startSolve();
        while (solving && !solveDone) solveStep();
    }

    void buildPath(int goal) {
        path.clear();
        int c = goal;
        while (c != -1) {
            path.push_back(c);
            c = cells[c].parent;
        }
        std::reverse(path.begin(), path.end());
    }

    int cols, rows;
    std::vector<Cell> cells;
    std::mt19937 rng;

    // generation state
    std::stack<int> genStack;
    bool genDone = false;
    int genCurrent = 0;

    // solving state
    bool solving = false;
    bool solveDone = false;
    std::queue<int> bfsQueue;
    std::vector<int> path;
    size_t pathIdx = 0;
};

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------
static void render(SDL_Renderer* ren, const Maze& maze, int cellSize, bool fastForward) {
    SDL_SetRenderDrawColor(ren, 18, 18, 24, 255);
    SDL_RenderClear(ren);

    int originX = MARGIN, originY = MARGIN;

    // fill visited-during-generation cells with a subtle background
    for (int r = 0; r < maze.rows; ++r) {
        for (int c = 0; c < maze.cols; ++c) {
            const Cell& cell = maze.cells[maze.idx(c, r)];
            SDL_Rect rect{ originX + c*cellSize, originY + r*cellSize, cellSize, cellSize };
            if (cell.bfsVisited) {
                SDL_SetRenderDrawColor(ren, 40, 60, 90, 255);
                SDL_RenderFillRect(ren, &rect);
            } else if (cell.visited) {
                SDL_SetRenderDrawColor(ren, 30, 30, 40, 255);
                SDL_RenderFillRect(ren, &rect);
            }
        }
    }

    // highlight the current DFS carve position while generating
    if (!maze.genDone) {
        int c = maze.colOf(maze.genCurrent), r = maze.rowOf(maze.genCurrent);
        SDL_Rect rect{ originX + c*cellSize, originY + r*cellSize, cellSize, cellSize };
        SDL_SetRenderDrawColor(ren, 90, 180, 90, 255);
        SDL_RenderFillRect(ren, &rect);
    }

    // start (top-left) and goal (bottom-right) markers
    {
        SDL_Rect s{ originX, originY, cellSize, cellSize };
        SDL_SetRenderDrawColor(ren, 60, 160, 60, 255);
        SDL_RenderFillRect(ren, &s);
        SDL_Rect g{ originX + (maze.cols-1)*cellSize, originY + (maze.rows-1)*cellSize, cellSize, cellSize };
        SDL_SetRenderDrawColor(ren, 200, 60, 60, 255);
        SDL_RenderFillRect(ren, &g);
    }

    // solved path
    if (!maze.path.empty()) {
        SDL_SetRenderDrawColor(ren, 250, 210, 60, 255);
        size_t drawUpTo = fastForward ? maze.path.size() : std::min(maze.pathIdx, maze.path.size());
        for (size_t i = 0; i < drawUpTo; ++i) {
            int c = maze.colOf(maze.path[i]), r = maze.rowOf(maze.path[i]);
            SDL_Rect rect{ originX + c*cellSize + cellSize/4, originY + r*cellSize + cellSize/4,
                           cellSize - cellSize/2, cellSize - cellSize/2 };
            SDL_RenderFillRect(ren, &rect);
        }
    }

    // walls
    SDL_SetRenderDrawColor(ren, 230, 230, 235, 255);
    for (int r = 0; r < maze.rows; ++r) {
        for (int c = 0; c < maze.cols; ++c) {
            const Cell& cell = maze.cells[maze.idx(c, r)];
            int x = originX + c*cellSize, y = originY + r*cellSize;
            if (cell.walls & WALL_N) SDL_RenderDrawLine(ren, x, y, x+cellSize, y);
            if (cell.walls & WALL_S) SDL_RenderDrawLine(ren, x, y+cellSize, x+cellSize, y+cellSize);
            if (cell.walls & WALL_E) SDL_RenderDrawLine(ren, x+cellSize, y, x+cellSize, y+cellSize);
            if (cell.walls & WALL_W) SDL_RenderDrawLine(ren, x, y, x, y+cellSize);
        }
    }

    // outer border (in case edge walls got visually thin)
    SDL_Rect border{ originX, originY, maze.cols*cellSize, maze.rows*cellSize };
    SDL_RenderDrawRect(ren, &border);

    SDL_RenderPresent(ren);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    (void)argc; (void)argv;
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window* win = SDL_CreateWindow("Maze Generator + Solver", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                        WINDOW_W, WINDOW_H, SDL_WINDOW_SHOWN);
    if (!win) {
        std::fprintf(stderr, "CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!ren) {
        std::fprintf(stderr, "CreateRenderer failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(win);
        SDL_Quit();
        return 1;
    }

    int cols = 30, rows = 30;
    int cellSize = (WINDOW_W - 2*MARGIN) / cols;
    Maze maze(cols, rows);

    bool running = true;
    bool fastForward = false;
    Uint32 lastStepTicks = SDL_GetTicks();
    const Uint32 STEP_INTERVAL_MS = 4; // pacing for animated carve/solve steps
    const int STEPS_PER_TICK = 3;      // how many carve/solve steps to do per interval

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;
            else if (e.type == SDL_KEYDOWN && e.key.repeat == 0) {
                switch (e.key.keysym.sym) {
                    case SDLK_ESCAPE: running = false; break;
                    case SDLK_SPACE:
                        maze.reset();
                        break;
                    case SDLK_RETURN:
                    case SDLK_KP_ENTER:
                        if (maze.genDone && maze.path.empty()) {
                            maze.startSolve();
                            if (fastForward) maze.solveInstant();
                        }
                        break;
                    case SDLK_f:
                        fastForward = !fastForward;
                        break;
                    case SDLK_EQUALS:
                    case SDLK_KP_PLUS:
                        cols = std::min(MAX_COLS, cols + 5);
                        rows = std::min(MAX_COLS, rows + 5);
                        cellSize = (WINDOW_W - 2*MARGIN) / cols;
                        maze = Maze(cols, rows);
                        break;
                    case SDLK_MINUS:
                    case SDLK_KP_MINUS:
                        cols = std::max(MIN_COLS, cols - 5);
                        rows = std::max(MIN_COLS, rows - 5);
                        cellSize = (WINDOW_W - 2*MARGIN) / cols;
                        maze = Maze(cols, rows);
                        break;
                    default: break;
                }
            }
        }

        if (fastForward) {
            if (!maze.genDone) maze.generateInstant();
            if (maze.solving && !maze.solveDone) maze.solveInstant();
        } else {
            Uint32 now = SDL_GetTicks();
            if (now - lastStepTicks >= STEP_INTERVAL_MS) {
                lastStepTicks = now;
                for (int i = 0; i < STEPS_PER_TICK; ++i) {
                    if (!maze.genDone) { maze.generateStep(); continue; }
                    if (maze.solving && !maze.solveDone) { maze.solveStep(); continue; }
                }
                // advance the path draw-in animation once solved
                if (maze.solveDone && maze.pathIdx < maze.path.size()) {
                    maze.pathIdx++;
                }
            }
        }
        if (fastForward) maze.pathIdx = maze.path.size();

        render(ren, maze, cellSize, fastForward);
        SDL_Delay(1);
    }

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}