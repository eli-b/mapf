using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Diagnostics;

namespace CPF_experiment
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

        public static readonly string GRID_NAME_KEY = "Grid Name";

        /// <summary>
        /// This contains extra data of this problem instance (used for special problem instances, e.g. subproblems of a bigger problem instance).
        /// </summary>
        public IDictionary<String, Object> parameters;

        /// <summary>
        /// Contains true at [x][y] if cell (x,y) is an obstacle
        /// </summary>
        public bool[][] m_vGrid;

        /// <summary>
        /// We keep a reference to the array of agents in the original problem.
        /// This will only change when IndependenceDetection's algorithm determines in another
        /// iteration that a new set of agents must be jointly planned due
        /// to their mutual conflicts.
        /// </summary>
        public AgentState[] m_vAgents;
        
        /// <summary>
        /// This is a matrix that contains the cost of the optimal path to the goal of every agent from any point in the grid.
        /// The first dimension of the matrix is the number of agents.
        /// The second dimension of the matrix is the cardinality of the location from which we want the shortest path.
        /// </summary>
        public int[][] singleAgentOptimalCosts;

        /// <summary>
        /// This is a matrix that contains the best move towards the goal of every agent from any point in the grid.
        /// The first dimension of the matrix is the number of agents.
        /// The second dimension of the matrix is the cardinality of the location from which we want the shortest path.
        /// </summary>
        public Move[][] singleAgentOptimalMoves;

        public uint m_nObstacles;
        public uint m_nLocations;
        
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
        public Int32[,] m_vCardinality;

        public ProblemInstance(IDictionary<String,Object> parameters = null)
        {
            if (parameters != null)
                this.parameters = parameters;
            else
                this.parameters = new Dictionary<String, Object>();
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
            subproblemInstance.Init(selectedAgents, this.m_vGrid, (int)this.m_nObstacles, (int)this.m_nLocations, this.m_vCardinality);
            subproblemInstance.singleAgentOptimalCosts = this.singleAgentOptimalCosts; // Each subproblem knows every agent's single shortest paths so this.singleAgentOptimalCosts[agent_num] would easily work
            subproblemInstance.singleAgentOptimalMoves = this.singleAgentOptimalMoves;
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
            m_vAgents = agentStartStates;
            m_vGrid = grid;
            
            if (nObstacles == -1)
                m_nObstacles = (uint)grid.Sum(row => row.Count(x => x));
            else
                m_nObstacles = (uint)nObstacles;

            if (nLocations == -1)
                m_nLocations = ((uint)(grid.Length * grid[0].Length)) - m_nObstacles;
            else
                m_nLocations = (uint)nLocations;
            
            if (cardinality == null)
                PrecomputeCardinality();
            else
                m_vCardinality = cardinality;
        }
        
        /// <summary>
        /// Compute the shortest path to the goal of every agent in the problem instance, from every location in the grid.
        /// Current implementation is a simple breadth-first search from every location in the graph.
        /// </summary>
        public void ComputeSingleAgentShortestPaths()
        {
            Debug.WriteLine("Computing the single agent shortest path for all agents...");
            //return; // Add for generator

            this.singleAgentOptimalCosts = new int[this.GetNumOfAgents()][];
            this.singleAgentOptimalMoves = new Move[this.GetNumOfAgents()][];

            for (int agentId = 0; agentId < this.GetNumOfAgents(); agentId++)
            {
                // Run a single source shortest path algorithm from the _goal_ of the agent
                var shortestPathLengths = new int[this.m_nLocations];
                var optimalMoves = new Move[this.m_nLocations];
                for (int i = 0; i < m_nLocations; i++)
                    shortestPathLengths[i] = -1;
                var openlist = new Queue<AgentState>();

                // Create initial state
                var agentStartState = this.m_vAgents[agentId];
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
                            int entry = m_vCardinality[aMove.x, aMove.y];
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
                }

                this.singleAgentOptimalCosts[agentId] = shortestPathLengths;
                this.singleAgentOptimalMoves[agentId] = optimalMoves;
            }
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
            return this.singleAgentOptimalCosts[agentNum][this.m_vCardinality[x, y]];
        }

        /// <summary>
        /// Returns the length of the shortest path between a given coordinate and the goal location of the given agent.
        /// </summary>
        /// <param name="agentNum"></param>
        /// <param name="move"></param>
        /// <returns>The length of the shortest path from x,y to the goal of the agent.</returns>
        public int GetSingleAgentOptimalCost(int agentNum, Move move)
        {
            return this.singleAgentOptimalCosts[agentNum][this.m_vCardinality[move.x, move.y]];
        }

        /// <summary>
        /// Returns the length of the shortest path between a given agent's location and the goal of that agent.
        /// </summary>
        /// <param name="agentState"></param>
        /// <returns>The length of the shortest path between a given agent's location and the goal of that agent</returns>
        public int GetSingleAgentOptimalCost(AgentState agentState)
        {
            return this.singleAgentOptimalCosts[agentState.agent.agentNum][this.m_vCardinality[agentState.lastMove.x, agentState.lastMove.y]];
        }

        /// <summary>
        /// Returns the optimal move towards the goal of the given agent. Move isn't necessarily unique.
        /// </summary>
        /// <param name="agentState"></param>
        /// <returns></returns>
        public Move GetSingleAgentOptimalMove(AgentState agentState)
        {
            return this.singleAgentOptimalMoves[agentState.agent.agentNum][this.m_vCardinality[agentState.lastMove.x, agentState.lastMove.y]];
        }

        /// <summary>
        /// The returned plan wasn't constructed considering a CAT, so it's possible there's an alternative plan with the same cost and less collisions.
        /// </summary>
        /// <param name="agentState"></param>
        /// <param name="conflictCountPerAgent"></param>
        /// <param name="conflictTimesPerAgent"></param>
        /// <returns></returns>
        public SinglePlan GetSingleAgentOptimalPlan(AgentState agentState,
                                                    out Dictionary<int, int> conflictCountPerAgent,
                                                    out Dictionary<int, List<int>> conflictTimesPerAgent)
        {
            LinkedList<Move> moves = new LinkedList<Move>();
            int agentNum = agentState.agent.agentNum;
            var conflictCounts = new Dictionary<int, int>();
            var conflictTimes = new Dictionary<int, List<int>>();
            IReadOnlyDictionary<TimedMove, List<int>> CAT;
            if (this.parameters.ContainsKey(CBS_LocalConflicts.CAT)) // TODO: Add support for IndependenceDetection's CAT
                CAT = ((IReadOnlyDictionary<TimedMove, List<int>>)this.parameters[CBS_LocalConflicts.CAT]);
            else
                CAT = new Dictionary<TimedMove, List<int>>();

            TimedMove current = agentState.lastMove; // The starting position
            int time = current.time;

            while (true)
            {
                moves.AddLast(current);

                // Count conflicts:
                current.UpdateConflictCounts(CAT, conflictCounts, conflictTimes);

                if (agentState.agent.Goal.Equals(current))
                    break;

                // Get next optimal move
                time++;
                Move optimal = this.singleAgentOptimalMoves[agentNum][this.GetCardinality(current)];
                current = new TimedMove(optimal, time);
            }

            conflictCountPerAgent = conflictCounts;
            conflictTimesPerAgent = conflictTimes;
            return new SinglePlan(moves, agentNum);
        }

        /// <summary>
        /// Utility function that returns the number of agents in this problem instance.
        /// </summary>
        public int GetNumOfAgents()
        {
            return m_vAgents.Length;
        }

        /// <summary>
        /// Utility function that returns the x dimension of the grid
        /// </summary>
        public int GetMaxX()
        {
            return this.m_vGrid.GetLength(0);
        }

        /// <summary>
        /// Utility function that returns the y dimension of the grid
        /// </summary>
        public int GetMaxY()
        {
            return this.m_vGrid[0].Length;
        }

        /// <summary>
        /// Imports a problem instance from a given file
        /// </summary>
        /// <param name="fileName"></param>
        /// <returns></returns>
        public static ProblemInstance Import(string fileName)
        {
            using (TextReader input = new StreamReader(fileName))
            {
                //return new ProblemInstance(); // DELETE ME!!!
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

                //instanceId = int.Parse(fileName.Split('-')[4]);
                // First/second line is Grid:
                Debug.Assert(line.StartsWith("Grid:"));

                // Read grid dimensions
                line = input.ReadLine();
                lineParts = line.Split(',');
                int maxX = int.Parse(lineParts[0]);
                int maxY = int.Parse(lineParts[1]);
                bool[][] grid = new bool[maxX][];
                char cell;
                for (int i = 0; i < maxX; i++)
                {
                    grid[i] = new bool[maxY];
                    line = input.ReadLine();
                    for (int j = 0; j < maxY; j++)
                    {
                        cell = line.ElementAt(j);
                        if (cell == '@' || cell == 'O' || cell == 'T' || cell == 'W' /* Water isn't traversable from land */)
                            grid[i][j] = true;
                        else
                            grid[i][j] = false;
                    }
                }

                // Next line is Agents:
                line = input.ReadLine();
                Debug.Assert(line.StartsWith("Agents:"));

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
                instance.parameters[ProblemInstance.GRID_NAME_KEY] = gridName;
                instance.ComputeSingleAgentShortestPaths();
                return instance;
            }
        }

        /// <summary>
        /// Exports a problem instance to a file
        /// </summary>
        /// <param name="fileName"></param>
        public void Export(string fileName)
        {
            TextWriter output = new StreamWriter(Directory.GetCurrentDirectory() + "\\Instances\\"+fileName);
            // Output the instance ID
            if (this.parameters.ContainsKey(ProblemInstance.GRID_NAME_KEY))
                output.WriteLine($"{this.instanceId},{this.parameters[ProblemInstance.GRID_NAME_KEY]}");
            else
                output.WriteLine(this.instanceId);

            // Output the grid
            output.WriteLine("Grid:");
            output.WriteLine($"{this.m_vGrid.GetLength(0)},{this.m_vGrid[0].GetLength(0)}");
                        
            for (int i = 0; i < this.m_vGrid.GetLength(0); i++)
            {
                for (int j = 0; j < this.m_vGrid[0].GetLength(0); j++)
                {
                    if (this.m_vGrid[i][j] == true)
                        output.Write('@');
                    else
                        output.Write('.');
                    
                }
                output.WriteLine();
            }
            // Output the agents state
            output.WriteLine("Agents:");
            output.WriteLine(this.m_vAgents.Length);
            AgentState state;
            for(int i = 0 ; i < this.m_vAgents.Length ; i++)
            {
                state = this.m_vAgents[i];
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
        /// <param name="location">An agent's current location.</param>
        /// <returns>n, where the agent is located at the nth non-obstacle
        /// location in our grid.</returns>
        public Int32 GetCardinality(Move location)
        {
            return m_vCardinality[location.x, location.y];
        }
        
        private void PrecomputeCardinality()
        {
            m_vCardinality = new Int32[m_vGrid.Length, m_vGrid[0].Length];
            Int32 maxCardinality = 0;
            for (uint i = 0; i < m_vGrid.Length; ++i)
                for (uint j = 0; j < m_vGrid[i].Length; ++j)
                {
                    if (m_vGrid[i][j])
                        m_vCardinality[i, j] = -1;
                    else
                        m_vCardinality[i, j] = maxCardinality++;
                }
        }

        /// <summary>
        /// Check if the tile is valid, i.e. in the grid and without an obstacle.
        /// NOT checking the direction. A Move could be declared valid even if it came to an edge tile from outside the grid!
        /// NOT checking if the move is illegal
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

            if (parameters.ContainsKey(IndependenceDetection.ILLEGAL_MOVES_KEY))
            {
                var reserved = (HashSet<TimedMove>)parameters[IndependenceDetection.ILLEGAL_MOVES_KEY];

                return (toCheck.IsColliding(reserved) == false);
            } // FIXME: Should this be here?

            return true;
        }

        public bool IsValidTile(int x, int y)
        {
            if (x < 0 || x >= GetMaxX())
                return false;
            if (y < 0 || y >= GetMaxY())
                return false;
            return !m_vGrid[x][y];
        }

        public override string ToString()
        {
            string str = $"Problem instance:{instanceId}";
            if (this.parameters.ContainsKey(ProblemInstance.GRID_NAME_KEY))
                str += $" Grid Name:{this.parameters[ProblemInstance.GRID_NAME_KEY]}";
            str += $" #Agents:{m_vAgents.Length}, GridCells:{m_nLocations}, #Obstacles:{m_nObstacles}";
            return str;
        }
    }
}
