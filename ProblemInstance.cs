using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Diagnostics;

namespace CPF_experement
{
    /// <summary>
    /// This class represents a cooperative pathfinding problem instance. This includes:
    /// - The grid in which the agent are locationed
    /// - An array of initial state for every agent.
    /// </summary>
    public class ProblemInstance
    {
        /// <summary>
        /// Delimiter used for export/import purposes
        /// </summary>
        private static char EXPORT_DELIMITER = ',';

        /// <summary>
        /// This contains extra data of this problem instance (used for special problem instances, e.g. subproblems of a bigger problem instance).
        /// </summary>
        public IDictionary<String, Object> parameters;

        public bool[][] m_vGrid;

        /// <summary>
        /// We keep a reference to the array of agents in the original problem.
        /// This will only change when Trevor's algorithm determines in another
        /// iteration that a new set of agents must be jointly planned due
        /// to their mutual conflicts.
        /// </summary>

        public AgentState[] m_vAgents;
        
        /// <summary>
        /// This is a matrix that contains the optimal path to the goal of every agent from any point in the grid.
        /// The first dimension of the matrix is the number of the agent.
        /// The second dimension of the matrix is the cardinality of the location from which we want the shortest path.
        /// </summary>
        public int[][] singleAgentShortestPaths;

        public uint m_nObstacles;
        public uint m_nLocations;
        public UInt64 m_nPermutations;
        public UInt64[] m_vPermutations;
        
        /// <summary>
        /// This field is used to identify an instance when running a set of experiments
        /// </summary>
        public int instanceId;
        
        /// <summary>
        /// Enumerates all of the empty spots in the grid. The indices
        /// correspond directly to those used in the grid, where the major
        /// index corresponds to the x-axis and the minor index corresponds to
        /// the y-axis.
        /// </summary>

        public Int32[,] m_vCardinality;

        public Int32 maxCardinality;

        public ProblemInstance()
        {
            this.parameters = new Dictionary<String, Object>();
        }

        /// <summary>
        /// Create a subproblem of this problem instance, in which only part of the agents are regarded.
        /// </summary>
        /// <param name="selectedAgents">The selected agent states that will be the root of the subproblem.</param>
        /// <returns></returns>
        public ProblemInstance Subproblem(AgentState[] selectedAgents)
        {
            ProblemInstance subproblemInstance = new ProblemInstance();
            subproblemInstance.init(selectedAgents, this.m_vGrid);
            subproblemInstance.singleAgentShortestPaths = this.singleAgentShortestPaths;
            return subproblemInstance;
        }

        /// <summary>
        /// Initialize the members of this object, such that the given agent states are the start state of this instance.
        /// </summary>
        /// <param name="agentStartStates"></param>
        /// <param name="grid"></param>
        public void init(AgentState[] agentStartStates, bool[][] grid)
        {
            m_vAgents = agentStartStates;
            m_vGrid = grid;
            m_nObstacles = 0;
            m_nLocations = 0;
            foreach (bool[] row in grid)
                foreach (bool bObstacle in row)
                    if (bObstacle)
                        ++m_nObstacles;
                    else
                        ++m_nLocations;
            precomputePermutations();
            precomputeCardinality();
        }
        
        /// <summary>
        /// Compute the shortest path to the goal of every agent in the problem instance, from every location in the grid.
        /// Current implementation is a simple breadth-first search from every location in the graph.
        /// </summary>
        public void ComputeSingleAgentShortestPaths()
        {
            Debug.WriteLine("Computing the single agent shortest path for all agents...");
            int[] shortestPaths;
            int entry;
            Move aMove;
            WorldState state, childState;
            AgentState agentState;
            AgentState currentAgentState;
            Queue<WorldState> openlist = new Queue<WorldState>();

            this.singleAgentShortestPaths = new int[this.GetNumOfAgents()][];
            for (int agentId = 0; agentId < this.GetNumOfAgents(); agentId++)
            {
                // Run a single source shortest path algorithm from the goal of the agent                
                shortestPaths = new int[this.m_nLocations];
                for (int i = 0; i < m_nLocations; i++)
                    shortestPaths[i] = -1;
                openlist.Clear();

                // Create initial state
                agentState = 
                    new AgentState(this.m_vAgents[agentId].agent.Goal_X,
                        this.m_vAgents[agentId].agent.Goal_Y,-1,-1,agentId);
                entry = this.getCardinality(agentState);
                shortestPaths[entry]=0;
                openlist.Enqueue(new WorldState(new AgentState[1] { agentState }));
                while (openlist.Count > 0)
                {
                    state = openlist.Dequeue();
                    currentAgentState = state.allAgentsState[0];

                    // Generate child states
                    for (int op = 0; op < WorldState.operators.GetLength(0); op++)
                    {
                        aMove = WorldState.MakeMove(op, currentAgentState);
                        if(IsValidForMove(aMove))
                        {
                            entry = m_vCardinality[aMove.x,aMove.y];
                            // If move will generate a new or better state - add it to the queue
                            if((shortestPaths[entry]<0)||(shortestPaths[entry]>state.g+1))
                            {
                                childState = new WorldState(state);
                                childState.allAgentsState[0].move(op);
                                childState.g=state.g+1;
                                shortestPaths[entry]=state.g+1;
                                openlist.Enqueue(childState);
                            }
                        }
                    }

                }
                this.singleAgentShortestPaths[agentId] = shortestPaths;
            }
        }

        /// <summary>
        /// Returns the shortest path between a given coordinates and the goal location of the given agent.
        /// </summary>
        /// <param name="agentNum"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns>The shortest path from x,y to the goal of agent agentNum</returns>
        public int GetSingleAgentShortestPath(int agentNum, int x, int y)
        {
            return this.singleAgentShortestPaths[agentNum][this.m_vCardinality[x,y]];
        }

        /// <summary>
        /// Returns the shortest path between a given coordinates and the goal location of the given agent.
        /// </summary>
        /// <param name="agentNum"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns>The shortest path from x,y to the goal of agent agentNum</returns>
        public int GetSingleAgentShortestPath(AgentState agent)
        {
            return this.singleAgentShortestPaths[agent.agent.agentNum][this.m_vCardinality[agent.pos_X, agent.pos_Y]];
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
        /// Roni: I am not sure when should this be used. It doesn't initalizes the grid, 
        /// so I assume that this is meant to be used when a single problem instance object is used and 
        /// modified during the search. This should be used with caution, as we are talking about references
        /// (so if one will change m_vAgents, all the other references to that instance will also point to the same, changed, instance.
        /// </summary>
        /// <param name="ags"></param>
        [Obsolete("Need to have some justification for using this. Currently I believe it will always cause bugs.")]
        public void init(AgentState[] ags)
        {
            m_vAgents = ags;
            precomputePermutations();
        }
        
        /// <summary>
        /// Imports a problem instance from a given file
        /// </summary>
        /// <param name="fileName"></param>
        /// <returns></returns>
        public static ProblemInstance Import(string fileName)
        {
            
            TextReader input = new StreamReader(fileName);
            string[] lineParts;
            string line;
            int instanceId=0;

            line = input.ReadLine();
            if (line.StartsWith("Grid:") == false)
            { 
                instanceId = int.Parse(line);
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
                    if (cell == '@')
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

            // Read the agent's start and goal states
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
            instance.init(states, grid);
            instance.instanceId = instanceId;
            instance.ComputeSingleAgentShortestPaths();
            return instance;
        }

        /// <summary>
        /// Exports a problem instance to a file
        /// </summary>
        /// <param name="fileName"></param>
        public void Export(string fileName)
        {
            TextWriter output = new StreamWriter(Directory.GetCurrentDirectory() + "\\Instances\\"+fileName);
            // Output the instance ID
            output.WriteLine(this.instanceId);

            // Output the grid
            output.WriteLine("Grid:");
            output.WriteLine(this.m_vGrid.GetLength(0)+","+this.m_vGrid[0].GetLength(0));
                        
            for (int i = 0; i < this.m_vGrid.GetLength(0); i++)
            {
                for (int j = 0; j < this.m_vGrid.GetLength(0); j++)
                {
                    if(this.m_vGrid[i][j]==true)
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
            for(int i=0;i<this.m_vAgents.Length;i++)
            {
                state = this.m_vAgents[i];
                output.Write(state.agent.agentNum);
                output.Write(EXPORT_DELIMITER);
                output.Write(state.agent.Goal_X);
                output.Write(EXPORT_DELIMITER);
                output.Write(state.agent.Goal_Y);
                output.Write(EXPORT_DELIMITER);
                output.Write(state.pos_X);
                output.Write(EXPORT_DELIMITER);
                output.Write(state.pos_Y);
                output.WriteLine();
            }
            output.Flush();
            output.Close();
        }

        /// <summary>
        /// Given an agent located at the nth location on our board that has
        /// not been occupied by an obstacle, we return n.
        /// </summary>
        /// <param name="ags">An agent's current location.</param>
        /// <returns>n, where the agent is located at the nth non-obstacle
        /// location in our grid.</returns>

        public Int32 getCardinality(AgentState ags)
        {
            return (m_vCardinality[ags.pos_X, ags.pos_Y]);
        }
        
        private void precomputePermutations()
        {
            m_vPermutations = new UInt64[m_vAgents.Length];
            m_vPermutations[m_vPermutations.Length - 1] = 1;
            for (int i = m_vPermutations.Length - 2; i >= 0; --i)
                m_vPermutations[i] = m_vPermutations[i + 1] * ((UInt64)(m_nLocations - (i + 1)));
            m_nPermutations = 1;
            uint nCurrentCounter = m_nLocations;
            for (uint i = 0; i < m_vAgents.Length; ++i)
            {
                m_nPermutations *= nCurrentCounter;
                --nCurrentCounter;
            }
            ++m_nPermutations;
        }
        
        private void precomputeCardinality()
        {
            m_vCardinality = new Int32[m_vGrid.Length, m_vGrid[0].Length];
            maxCardinality = 0;
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
        /// Check if the tile is valid, i.e. in the grid and without an obstacle
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns>True if the give location is a valid grid location with no obstacles</returns>
        public bool IsValidForMove(Move aMove)
        {
            int x = aMove.x;
            int y = aMove.y;

            if ((x >= m_vGrid.Length) || (x < 0))
            {
                return false;
            }
            if ((y >= m_vGrid[x].Length) || (y < 0))
            {
                return false;
            }
            return !m_vGrid[x][y];
        }

        /// <summary>
        /// Check if the tile is valid, i.e. in the grid and without an obstacle and in sum cases that the step is not reserved
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns>True if the give location is a valid grid location with no obstacles</returns>
        public bool IsValidForMove(int x, int y, int time = 0, int direction = -1)
        {
            if ((x >= m_vGrid.Length) || (x < 0))
                return false;
            if ((y >= m_vGrid[x].Length) || (y < 0))
                return false;
            if (m_vGrid[x][y] == true)
                return false;
            //if (parameters.ContainsKey(Trevor.ILLEGAL_MOVES_KEY))
            //{
            //    Plan reserved = (Plan)parameters[Trevor.ILLEGAL_MOVES_KEY];
            //    LinkedList<Move> reservedAtTime = reserved.GetLocationsAt(time);
            //    foreach (Move res in reservedAtTime)
            //    {
            //        if (res.isColliding(x, y, direction))
            //            return false;
            //    }
            //}
            return true;
        }

        public bool IsValidForMove(TimedMove toCheck)
        {
            if ((toCheck.x >= m_vGrid.Length) || (toCheck.x < 0))
                return false;
            if ((toCheck.y >= m_vGrid[toCheck.x].Length) || (toCheck.y < 0))
                return false;
            if (m_vGrid[toCheck.x][toCheck.y] == true)
                return false;
            if (parameters.ContainsKey(Trevor.ILLEGAL_MOVES_KEY))
            {
                HashSet<TimedMove> reserved = (HashSet<TimedMove>)parameters[Trevor.ILLEGAL_MOVES_KEY];
                int direction = toCheck.direction;
                toCheck.direction = -1;
                if (reserved.Contains(toCheck))
                    return false;
                toCheck.direction = direction;
                toCheck.setOppositeMove();
                if (reserved.Contains(toCheck))
                    return false;
                toCheck.setOppositeMove();
            }
            //if (parameters.ContainsKey(CBS_LocalConflicts.CONSTRAINTS))
            //{
            //    DnCConstraint nextStepLocation = new DnCConstraint();
            //    nextStepLocation.init(-1, toCheck.x, toCheck.y, toCheck.time, toCheck.direction);
            //    if (((HashSet_U<DnCConstraint>)parameters[CBS_LocalConflicts.CONSTRAINTS]).Contains(nextStepLocation))
            //        return false;
            //}
            return true;
        }

        public bool isValidTile(int x, int y)
        {
            if (x < 0 || x >= GetMaxX())
                return false;
            if (y < 0 || y >= GetMaxY())
                return false;
            return !m_vGrid[x][y];
        }

        public override string ToString()
        {
            return "Problem instace:"+instanceId+" #Agents:" + m_vAgents.Length
                    + ", GridCells:" + m_nLocations + ", #Obstacles:" + m_nObstacles;                
        }

        public double getConflictRation(int orderOfConflict)
        {
            HashSet<CoordinateForConflictRatio> potentialConflicts = new HashSet<CoordinateForConflictRatio>();
            SetPotentialConflictsForAgent set=new SetPotentialConflictsForAgent();
            int conflictsCount = 0;
            int clearCount = 0;


            for (int i = 0; i < m_vAgents.Length; i++)
            {
                set.Setup(this, i);
                set.Solve(potentialConflicts, orderOfConflict);
                conflictsCount += set.conflictsCount;
                if(i!=0)
                    clearCount += set.clearCount;
            }

            return conflictsCount;
        }

       
    }
}
