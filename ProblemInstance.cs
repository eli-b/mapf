using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Diagnostics;

namespace mapf
{
    /// <summary>
    /// This class represents a cooperative pathfinding problem instance. This includes:
    /// - The grid in which the agents are located
    /// - An array of initial state for every agent.
    /// </summary>
    public class ProblemInstance
    {
        /// <summary>
        /// Delimiter used for export/import purposes
        /// </summary>
        private static readonly char EXPORT_DELIMITER = ',';

        /// <summary>
        /// This contains extra data of this problem instance (used for special problem instances, e.g. subproblems of a bigger problem instance).
        /// </summary>
        public IDictionary<string, object> parameters;

        /// <summary>
        /// Contains true at [x][y] if cell (x,y) is an obstacle
        /// </summary>
        public bool[][] grid;

        /// <summary>
        /// We keep a reference to the array of agents in the original problem.
        /// This will only change when IndependenceDetection's algorithm determines in another
        /// iteration that a new set of agents must be jointly planned due
        /// to their mutual conflicts.
        /// </summary>
        public AgentState[] agents;

        /// <summary>
        /// This is a matrix that contains the cost of the optimal path to the goal of every agent from any point in the grid.
        /// The first dimension of the matrix is the number of agents.
        /// The second dimension of the matrix is the cardinality of the location from which we want the shortest path.
        /// </summary>
        public int[][] singleAgentOptimalCosts;

        /// <summary>
        /// The time it took to compute the shortest paths.
        /// </summary>
        public double shortestPathComputeTime;

        /// <summary>
        /// This is a matrix that contains the best move towards the goal of every agent from any point in the grid.
        /// The first dimension of the matrix is the number of agents.
        /// The second dimension of the matrix is the cardinality of the location from which we want the shortest path.
        /// </summary>
        public Move[][] singleAgentOptimalMoves;

        public uint numObstacles;
        public uint numLocations;
        
        /// <summary>
        /// This field is used to identify an instance when running a set of experiments
        /// </summary>
        public int instanceId;
        
        /// <summary>
        /// Enumerates all of the empty spots in the grid. The indices
        /// correspond directly to those used in the grid, where the major
        /// index corresponds to the x-axis and the minor index corresponds to
        /// the y-axis. If there are obstacles, it's more space-efficient to store
        /// data for each non-empty spot.
        /// </summary>
        public int[,] cardinality;

        public string gridName;
        public string instanceName;

        public ProblemInstance(IDictionary<string, object> parameters = null)
        {
            if (parameters != null)
                this.parameters = parameters;
            else
                this.parameters = new Dictionary<string, object>();
        }

        /// <summary>
        /// Create a subproblem of this problem instance, in which only part of the agents are regarded.
        /// </summary>
        /// <param name="selectedAgents">The selected agent states that will be the root of the subproblem.</param>
        /// <returns></returns>
        public ProblemInstance Subproblem(AgentState[] selectedAgents)
        {
            // Notice selected agents may actually be a completely different set of agents.
            // Not copying instance id. This isn't the same problem.
            ProblemInstance subproblemInstance = new ProblemInstance(this.parameters);
            subproblemInstance.Init(selectedAgents, this.grid, (int)this.numObstacles, (int)this.numLocations, this.cardinality);
            subproblemInstance.singleAgentOptimalCosts = this.singleAgentOptimalCosts; // Each subproblem knows every agent's single shortest paths so this.singleAgentOptimalCosts[agent_num] would easily work
            subproblemInstance.singleAgentOptimalMoves = this.singleAgentOptimalMoves;
            subproblemInstance.gridName = this.gridName;
            subproblemInstance.instanceName = this.instanceName;
            subproblemInstance.instanceId = this.instanceId;
            return subproblemInstance;
        }

        /// <summary>
        /// Initialize the members of this object, such that the given agent states are the start state of this instance.
        /// </summary>
        /// <param name="agentStartStates"></param>
        /// <param name="grid"></param>
        /// <param name="nObstacles"></param>
        /// <param name="nLocations"></param>
        /// <param name="cardinality"></param>
        public void Init(AgentState[] agentStartStates, bool[][] grid, int nObstacles=-1,
                         int nLocations=-1, int[,] cardinality=null)
        {
            agents = agentStartStates;
            this.grid = grid;
            
            if (nObstacles == -1)
                numObstacles = (uint)grid.Sum(row => row.Count(x => x));
            else
                numObstacles = (uint)nObstacles;

            if (nLocations == -1)
                numLocations = ((uint)(grid.Length * grid[0].Length)) - numObstacles;
            else
                numLocations = (uint)nLocations;
            
            if (cardinality == null)
                PrecomputeCardinality();
            else
                this.cardinality = cardinality;
        }
        
        /// <summary>
        /// Compute the shortest path to the goal of every agent in the problem instance, from every location in the grid.
        /// Current implementation is a simple breadth-first search from every location in the graph.
        /// </summary>
        public void ComputeSingleAgentShortestPaths()
        {
            Debug.WriteLine("Computing the single agent shortest path for all agents...");
            Stopwatch watch = Stopwatch.StartNew();
            double startTime = watch.Elapsed.TotalMilliseconds;
            //return; // Add for generator

            this.singleAgentOptimalCosts = new int[this.GetNumOfAgents()][];
            this.singleAgentOptimalMoves = new Move[this.GetNumOfAgents()][];

            for (int agentId = 0; agentId < this.GetNumOfAgents(); agentId++)
            {
                // Run a single source shortest path algorithm from the _goal_ of the agent
                var shortestPathLengths = new int[this.numLocations];
                var optimalMoves = new Move[this.numLocations];
                for (int i = 0; i < numLocations; i++)
                    shortestPathLengths[i] = -1;
                var openlist = new Queue<AgentState>();

                // Create initial state
                var agentStartState = this.agents[agentId];
                var agent = agentStartState.agent;
                var goalState = new AgentState(agent.Goal.x, agent.Goal.y, -1, -1, agentId);
                int goalIndex = this.GetCardinality(goalState.lastMove);
                shortestPathLengths[goalIndex] = 0;
                optimalMoves[goalIndex] = new Move(goalState.lastMove);
                openlist.Enqueue(goalState);

                while (openlist.Count > 0)
                {
                    AgentState state = openlist.Dequeue();

                    // Generate child states
                    foreach (TimedMove aMove in state.lastMove.GetNextMoves())
                    {
                        if (IsValid(aMove))
                        {
                            int entry = cardinality[aMove.x, aMove.y];
                            // If move will generate a new or better state - add it to the queue
                            if ((shortestPathLengths[entry] == -1) || (shortestPathLengths[entry] > state.g + 1))
                            {
                                var childState = new AgentState(state);
                                childState.MoveTo(aMove);
                                shortestPathLengths[entry] = childState.g;
                                optimalMoves[entry] = new Move(aMove.GetOppositeMove());
                                openlist.Enqueue(childState);
                            }
                        }
                    }

                }

                int start = this.GetCardinality(agentStartState.lastMove);
                if (shortestPathLengths[start] == -1)
                {
                    throw new Exception($"Unsolvable instance! Agent {agentId} cannot reach its goal");
                    // Note instances can still be unsolvable if this isn't reached. E.g. this corridor:
                    // s1-g2-g1-s2
                }

                this.singleAgentOptimalCosts[agentId] = shortestPathLengths;
                this.singleAgentOptimalMoves[agentId] = optimalMoves;
            }
            double endTime = watch.Elapsed.TotalMilliseconds;
            this.shortestPathComputeTime = endTime - startTime;
        }

        /// <summary>
        /// Returns the length of the shortest path between a given coordinate and the goal location of the given agent.
        /// </summary>
        /// <param name="agentNum"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns>The length of the shortest path from x,y to the goal of the agent.</returns>
        public int GetSingleAgentOptimalCost(int agentNum, int x, int y)
        {
            return this.singleAgentOptimalCosts[agentNum][this.cardinality[x, y]];
        }

        /// <summary>
        /// Returns the length of the shortest path between a given coordinate and the goal location of the given agent.
        /// </summary>
        /// <param name="agentNum"></param>
        /// <param name="move"></param>
        /// <returns>The length of the shortest path from x,y to the goal of the agent.</returns>
        public int GetSingleAgentOptimalCost(int agentNum, Move move)
        {
            return this.singleAgentOptimalCosts[agentNum][this.cardinality[move.x, move.y]];
        }

        /// <summary>
        /// Returns the length of the shortest path between a given agent's location and the goal of that agent.
        /// </summary>
        /// <param name="agentState"></param>
        /// <returns>The length of the shortest path between a given agent's location and the goal of that agent</returns>
        public int GetSingleAgentOptimalCost(AgentState agentState)
        {
            int locationCardinality = this.cardinality[agentState.lastMove.x, agentState.lastMove.y];
            return this.singleAgentOptimalCosts[agentState.agent.agentNum][locationCardinality];
        }

        /// <summary>
        /// Returns the optimal move towards the goal of the given agent. Move isn't necessarily unique.
        /// </summary>
        /// <param name="agentState"></param>
        /// <returns></returns>
        public Move GetSingleAgentOptimalMove(AgentState agentState)
        {
            int locationCardinality = this.cardinality[agentState.lastMove.x, agentState.lastMove.y];
            return this.singleAgentOptimalMoves[agentState.agent.agentNum][locationCardinality];
        }

        /// <summary>
        /// Note: The returned plan wasn't constructed considering a CAT, so it's possible there's an alternative plan with the same cost and less collisions.
        /// </summary>
        /// <param name="agentState"></param>
        /// <returns>An optimal plan for the agent, ignoring all others</returns>
        public SinglePlan GetSingleAgentOptimalPlan(AgentState agentState)
        {
            LinkedList<Move> moves = new LinkedList<Move>();
            int agentNum = agentState.agent.agentNum;
            TimedMove current = agentState.lastMove; // The starting position
            int time = current.time;

            while (true)
            {
                moves.AddLast(current);

                if (agentState.agent.Goal.Equals(current))
                    break;

                // Get next optimal move
                time++;
                Move optimal = this.singleAgentOptimalMoves[agentNum][this.GetCardinality(current)];
                current = new TimedMove(optimal, time);
            }

            return new SinglePlan(moves, agentNum);
        }

        /// <summary>
        /// Utility function that returns the number of agents in this problem instance.
        /// </summary>
        public int GetNumOfAgents()
        {
            return this.agents.Length;
        }

        /// <summary>
        /// Utility function that returns the x dimension of the grid
        /// </summary>
        public int GetMaxX()
        {
            return this.grid.Length;
        }

        /// <summary>
        /// Utility function that returns the y dimension of the grid
        /// </summary>
        public int GetMaxY()
        {
            return this.grid[0].Length;
        }

        private static bool[][] readMapFile(string mapFilePath)
        {
            using (TextReader input = new StreamReader(mapFilePath))
            {
                // Read grid dimensions
                string line = input.ReadLine();
                if (line == "type octile")
                    return readBenchmarkMap(input, line);
                else
                    return readLironMap(input, line);
            }
        }

        private static bool[][] readBenchmarkMap(TextReader input, string line)
        {
            bool[][] grid;
            string[] lineParts;
            int maxX, maxY;
            // Read grid dimensions
            line = input.ReadLine();
            lineParts = line.Split(' ');
            Trace.Assert(lineParts.Length == 2);
            Trace.Assert(lineParts[0].Equals("height"));
            maxY = int.Parse(lineParts[1]);  // The height is the number of rows
            line = input.ReadLine();
            lineParts = line.Split(' ');
            Trace.Assert(lineParts.Length == 2);
            Trace.Assert(lineParts[0].Equals("width"));
            maxX = int.Parse(lineParts[1]);  // The width is the number of columns
            grid = new bool[maxY][];

            line = input.ReadLine();
            Trace.Assert(line.StartsWith("map"));

            char cell;
            // Read grid
            for (int i = 0; i < maxY; i++)
            {
                grid[i] = new bool[maxX];
                line = input.ReadLine();
                for (int j = 0; j < maxX; j++)
                {
                    cell = line[j];
                    if (cell == '@' || cell == 'O' || cell == 'T' || cell == 'W' /* Water isn't traversable from land */)
                        grid[i][j] = true;
                    else
                        grid[i][j] = false;
                }
            }
            return grid;
        }


        private static bool[][] readLironMap(TextReader input, string line) {
            string[] lineParts;
            lineParts = line.Split(',');
            int maxX = int.Parse(lineParts[0]);
            int maxY = int.Parse(lineParts[1]);
            bool[][] grid = new bool[maxX][];
            char cell;
            // Read grid
            for (int i = 0; i < maxX; i++)
            {
                grid[i] = new bool[maxY];
                line = input.ReadLine();
                for (int j = 0; j < maxY; j++)
                {
                    cell = line[j];
                    if (cell == '1')
                        grid[i][j] = true;
                    else
                        grid[i][j] = false;
                }
            }
            return grid;
        }

        /// <summary>
        /// Imports a problem instance from a given file
        /// </summary>
        /// <param name="filePath"></param>
        /// <param name="mapFilePath"></param>
        /// <returns></returns>
        public static ProblemInstance Import(string filePath, string mapFilePath = null)
        {
            if (filePath.EndsWith(".agents"))
            {
                string fileNameWithoutExtension = Path.GetFileNameWithoutExtension(filePath);
                int instanceId = 0;
                try
                {
                    instanceId = int.Parse(filePath.Split('_').Last());
                }
                catch (Exception) {}
                string mapfileNameWithoutExtension;
                if (mapFilePath == null)
                {
                    mapfileNameWithoutExtension = fileNameWithoutExtension.Substring(0, length: fileNameWithoutExtension.LastIndexOf('_')) + ".map";  // Passing a length parameter is like specifying a non-inclusive end index
                    mapFilePath = Path.Combine(Path.GetDirectoryName(filePath), "..", "..", "..", "maps", mapfileNameWithoutExtension);
                }
                else
                {
                    mapfileNameWithoutExtension = Path.GetFileNameWithoutExtension(mapFilePath);
                }

                bool[][] grid = readMapFile(mapFilePath);

                string line;
                string[] lineParts;
                AgentState[] states;
                using (TextReader input = new StreamReader(filePath))
                {
                    // Read the number of agents
                    line = input.ReadLine();
                    int numOfAgents = int.Parse(line);

                    // Read the agents' start and goal states
                    states = new AgentState[numOfAgents];
                    AgentState state;
                    Agent agent;
                    int goalX;
                    int goalY;
                    int startX;
                    int startY;
                    for (int i = 0; i < numOfAgents; i++)
                    {
                        line = input.ReadLine();
                        lineParts = line.Split(EXPORT_DELIMITER);
                        goalX = int.Parse(lineParts[0]);
                        goalY = int.Parse(lineParts[1]);
                        startX = int.Parse(lineParts[2]);
                        startY = int.Parse(lineParts[3]);
                        agent = new Agent(goalX, goalY, i);
                        state = new AgentState(startX, startY, agent);
                        states[i] = state;
                    }
                }

                // Generate the problem instance
                ProblemInstance instance = new ProblemInstance();
                instance.Init(states, grid);
                instance.instanceId = instanceId;
                instance.gridName = mapfileNameWithoutExtension;
                instance.instanceName = fileNameWithoutExtension + ".agents";
                instance.ComputeSingleAgentShortestPaths();
                return instance;
            }
            else if (filePath.EndsWith(".scen"))
            {
                string fileNameWithoutExtension = Path.GetFileNameWithoutExtension(filePath);
                int instanceId = int.Parse(fileNameWithoutExtension.Split('-').Last());
                string mapfileName = fileNameWithoutExtension.Substring(0, length: fileNameWithoutExtension.LastIndexOf('-'));  // Passing a length parameter is like specifying a non-inclusive end index
                if (mapFilePath == null)
                    mapFilePath = Path.Combine(Path.GetDirectoryName(filePath), "..", "..", "maps", mapfileName);
                
                bool[][] grid = readMapFile(mapFilePath);

                string line;
                string[] lineParts;
                List<AgentState> stateList = new List<AgentState>();
                using (TextReader input = new StreamReader(filePath))
                {
                    // Read the format version number
                    line = input.ReadLine();
                    lineParts = line.Split(' ');
                    Trace.Assert(lineParts[0].Equals("version"));
                    int version = int.Parse(lineParts[1]);
                    Trace.Assert(version == 1, "Only version 1 is currently supported");

                    // Read the agents' start and goal states
                    AgentState state;
                    Agent agent;
                    int agentNum = 0;
                    int block;
                    int goalX;
                    int goalY;
                    int startX;
                    int startY;
                    string mapFileName;
                    int mapRows;
                    int mapCols;
                    double optimalCost;  // Assuming diagonal moves are allowed and cost sqrt(2)
                    while (true)
                    {
                        line = input.ReadLine();
                        if (string.IsNullOrWhiteSpace(line))
                            break;
                        lineParts = line.Split('\t');
                        block = int.Parse(lineParts[0]);
                        mapFileName = lineParts[1];
                        mapRows = int.Parse(lineParts[3]);
                        Trace.Assert(mapRows == grid.Length);
                        mapCols = int.Parse(lineParts[2]);
                        Trace.Assert(mapCols == grid[0].Length);

                        startY = int.Parse(lineParts[4]);
                        startX = int.Parse(lineParts[5]);
                        goalY = int.Parse(lineParts[6]);
                        goalX = int.Parse(lineParts[7]);
                        optimalCost = double.Parse(lineParts[8]);
                        agent = new Agent(goalX, goalY, agentNum);
                        state = new AgentState(startX, startY, agent);
                        stateList.Add(state);
                        agentNum++;
                    }
                }

                // Generate the problem instance
                ProblemInstance instance = new ProblemInstance();
                instance.Init(stateList.ToArray(), grid);
                instance.instanceId = instanceId;
                instance.gridName = mapfileName;
                instance.instanceName = fileNameWithoutExtension + ".scen";
                instance.ComputeSingleAgentShortestPaths();
                return instance;
            }
            else  // Combined map and scenario, no suffix
            {
                using (TextReader input = new StreamReader(filePath))
                {
                    string[] lineParts;
                    string line;
                    int instanceId = 0;
                    string gridName = "Random Grid"; // The default

                    line = input.ReadLine();
                    if (line.StartsWith("Grid:") == false)
                    {
                        lineParts = line.Split(',');
                        instanceId = int.Parse(lineParts[0]);
                        if (lineParts.Length > 1)
                            gridName = lineParts[1];
                        line = input.ReadLine();
                    }

                    // First/second line is Grid:
                    Trace.Assert(line.StartsWith("Grid:"));

                    // Read grid dimensions
                    line = input.ReadLine();
                    lineParts = line.Split(',');
                    int maxX = int.Parse(lineParts[0]);
                    int maxY = int.Parse(lineParts[1]);
                    bool[][] grid = new bool[maxX][];
                    // Read grid
                    char cell;
                    for (int i = 0; i < maxX; i++)
                    {
                        grid[i] = new bool[maxY];
                        line = input.ReadLine();
                        for (int j = 0; j < maxY; j++)
                        {
                            cell = line[j];
                            if (cell == '@' || cell == 'O' || cell == 'T' || cell == 'W' /* Water isn't traversable from land */)
                                grid[i][j] = true;
                            else
                                grid[i][j] = false;
                        }
                    }

                    // Next line is Agents:
                    line = input.ReadLine();
                    Trace.Assert(line.StartsWith("Agents:"));

                    // Read the number of agents
                    line = input.ReadLine();
                    int numOfAgents = int.Parse(line);

                    // Read the agents' start and goal states
                    AgentState[] states = new AgentState[numOfAgents];
                    AgentState state;
                    Agent agent;
                    int agentNum;
                    int goalX;
                    int goalY;
                    int startX;
                    int startY;
                    for (int i = 0; i < numOfAgents; i++)
                    {
                        line = input.ReadLine();
                        lineParts = line.Split(EXPORT_DELIMITER);
                        agentNum = int.Parse(lineParts[0]);
                        goalX = int.Parse(lineParts[1]);
                        goalY = int.Parse(lineParts[2]);
                        startX = int.Parse(lineParts[3]);
                        startY = int.Parse(lineParts[4]);
                        agent = new Agent(goalX, goalY, agentNum);
                        state = new AgentState(startX, startY, agent);
                        states[i] = state;
                    }

                    // Generate the problem instance
                    ProblemInstance instance = new ProblemInstance();
                    instance.Init(states, grid);
                    instance.instanceId = instanceId;
                    instance.gridName = gridName;
                    instance.instanceName = Path.GetFileNameWithoutExtension(filePath);
                    instance.ComputeSingleAgentShortestPaths();
                    return instance;
                }
            }
        }

        /// <summary>
        /// Exports a problem instance to a file
        /// </summary>
        /// <param name="fileName"></param>
        public void Export(string fileName)
        {
            TextWriter output = new StreamWriter(Path.Combine(Directory.GetCurrentDirectory(), "..", "..", "..", "Instances", fileName));
            // Output the instance ID
            output.WriteLine($"{this.instanceId},{this.gridName}");

            // Output the grid
            output.WriteLine("Grid:");
            output.WriteLine($"{this.grid.Length},{this.grid[0].Length}");
                        
            for (int i = 0; i < this.grid.Length; i++)
            {
                for (int j = 0; j < this.grid[0].Length; j++)
                {
                    if (this.grid[i][j] == true)
                        output.Write('@');
                    else
                        output.Write('.');
                    
                }
                output.WriteLine();
            }
            // Output the agents state
            output.WriteLine("Agents:");
            output.WriteLine(this.agents.Length);
            AgentState state;
            for(int i = 0 ; i < this.agents.Length ; i++)
            {
                state = this.agents[i];
                output.Write(state.agent.agentNum);
                output.Write(EXPORT_DELIMITER);
                output.Write(state.agent.Goal.x);
                output.Write(EXPORT_DELIMITER);
                output.Write(state.agent.Goal.y);
                output.Write(EXPORT_DELIMITER);
                output.Write(state.lastMove.x);
                output.Write(EXPORT_DELIMITER);
                output.Write(state.lastMove.y);
                output.WriteLine();
            }
            output.Flush();
            output.Close();
        }

        /// <summary>
        /// Given an agent located at the nth location on our board that is
        /// not occupied by an obstacle, we return n.
        /// </summary>
        /// <param name="move">An agent's current location.</param>
        /// <returns>n, where the agent is located at the nth non-obstacle
        /// location in our grid.</returns>
        public int GetCardinality(Move move)
        {
            return cardinality[move.x, move.y];
        }
        
        private void PrecomputeCardinality()
        {
            cardinality = new int[grid.Length, grid[0].Length];
            int maxCardinality = 0;
            for (uint i = 0; i < grid.Length; ++i)
                for (uint j = 0; j < grid[i].Length; ++j)
                {
                    if (grid[i][j])
                        cardinality[i, j] = -1;
                    else
                        cardinality[i, j] = maxCardinality++;
                }
        }

        /// <summary>
        /// Check if the tile is valid, i.e. in the grid and without an obstacle.
        /// NOT checking the direction. A Move could be declared valid even if it came to an edge tile from outside the grid!
        /// </summary>
        /// <param name="aMove"></param>
        /// <returns>True if the given location is a valid grid location with no obstacles</returns>
        public bool IsValid(Move aMove)
        {
            return IsValidTile(aMove.x, aMove.y);
        }

        /// <summary>
        /// Also checks if the move is illegal
        /// </summary>
        /// <param name="toCheck"></param>
        /// <returns></returns>
        public bool IsValid(TimedMove toCheck)
        {
            if (IsValidTile(toCheck.x, toCheck.y) == false)
                return false;

            return true;
        }

        public bool IsValidTile(int x, int y)
        {
            if (x < 0 || x >= GetMaxX())
                return false;
            if (y < 0 || y >= GetMaxY())
                return false;
            return !grid[x][y];
        }

        public override string ToString()
        {
            return $"Problem instance name:{instanceName} #Agents:{agents.Length}, GridCells:{numLocations}, #Obstacles:{numObstacles}";
        }
    }
}
