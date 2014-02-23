using System.Collections.Generic;
using System.Linq;


namespace CPF_experiment
{
    /// <summary>
    /// A* implementation with Standley's operator decomposition (OD). See AAAI 2010 paper by Trevor Scott Standley on Cooperative Pathfinding.
    /// </summary>
    public class AStarWithOD : ClassicAStar
    {
        override public void Setup(ProblemInstance problemInstance)
        {
            base.Setup(problemInstance);
        }

        override protected WorldState CreateSearchRoot()
        {
            WorldStateWithOD root = new WorldStateWithOD(this.instance.m_vAgents);
           // root.makespan = 1;
            return root;
        }

        override public string GetName() { return "A*+OD"; }

        /// <summary>
        /// Expand a given node. This includes:
        /// - Generating all possible children
        /// - Inserting them to OPEN
        /// - Insert the generated nodes to the hashtable of nodes, currently implmented together with the closed list.
        /// </summary>
        override public bool Expand(WorldState node)
        {
            if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                return false;
            if (((WorldStateWithOD)node).agentTurn == 0)
                expandedFullStates++;
            //Debug.Print("Expanding node " + node);
            WorldStateWithOD parent = (WorldStateWithOD)node;
            int deltaX;
            int deltaY;
            int agentTurn = parent.agentTurn;
            int posX = parent.allAgentsState[agentTurn].pos_X;
            int posY = parent.allAgentsState[agentTurn].pos_Y;
            WorldStateWithOD childNode;
            TimedMove newMove=new TimedMove();
            int childAgentTurn = ((parent.agentTurn + 1) % (this.instance.m_vAgents.Length));
            int step = node.makespan;
            if (parent.agentTurn == 0)
                step++;
            
            // Try all legal moves of the agents
            for (int op = 0; op < WorldState.operators.GetLength(0); op++)//{{0,0,0},{-1,0,1},{0,1,2},{1,0,3},{0,-1,4}}
            {
               // this.generated++;
                deltaX = WorldState.operators[op, 0];
                deltaY = WorldState.operators[op, 1];
                newMove.setup(posX + deltaX, posY + deltaY, WorldState.operators[op, 2],step);
                if (this.IsValid(newMove, agentTurn, parent))
                {
                  
                    childNode = new WorldStateWithOD(parent);
                    childNode.allAgentsState[agentTurn].move(op);
                    childNode.prevStep = parent;
                    childNode.agentTurn = childAgentTurn;
                    childNode.h = (int)this.heuristic.h(childNode);

                    if (instance.parameters.ContainsKey(Trevor.CONFLICT_AVOIDENCE))
                    {
                        childNode.potentialConflictsCount = parent.potentialConflictsCount;
                        childNode.potentialConflictsCount += childNode.conflictsCount(((HashSet<TimedMove>)instance.parameters[Trevor.CONFLICT_AVOIDENCE]));
                    }

                    if (instance.parameters.ContainsKey(CBS_LocalConflicts.INTERNAL_CAT))
                    {
                        childNode.dncInternalConflictsCount = parent.prevStep.dncInternalConflictsCount;
                        childNode.dncInternalConflictsCount += parent.conflictsCount(((HashSet<TimedMove>)instance.parameters[CBS_LocalConflicts.INTERNAL_CAT]));
                    }

                    // Makespan increases only if this is the move of the first agent
                    if (parent.agentTurn == 0)
                        childNode.makespan = parent.makespan + 1;


                    // g of child is equal to g of parent only when newMove is a STAY move and agent has already arrived at his goal location
                    childNode.CalculateG();  

                    if (childNode.h + childNode.g <= this.maxCost)
                    {
                        //if in closed list
                        //if (childNode.agentTurn==0 && this.closedList.Contains(childNode) == true)
                        if (this.closedList.ContainsKey(childNode) == true)
                        {
                            WorldStateWithOD inClosedList = (WorldStateWithOD)this.closedList[childNode];
                            if (inClosedList.mirrorState != null)
                                inClosedList = inClosedList.mirrorState;
                            //if g is smaller than remove the old world state than remove state
                            if (inClosedList != null && (inClosedList.g > childNode.g || (inClosedList.g == childNode.g && (inClosedList.potentialConflictsCount > childNode.potentialConflictsCount || (inClosedList.potentialConflictsCount == childNode.potentialConflictsCount && inClosedList.dncInternalConflictsCount > childNode.dncInternalConflictsCount)))))
                            {
                                closedList.Remove(inClosedList);
                                openList.Remove(inClosedList);
                            }
                        }
                        //if (childNode.agentTurn != 0 || (childNode.agentTurn == 0 && this.closedList.Contains(childNode) == false))
                        if (this.closedList.ContainsKey(childNode) == false)
                        {
                            this.generated++;
                            //if (childNode.agentTurn == 0)
                            //    closedList.Add(childNode);
                            this.addToClosedList(childNode);
                            if (childNode.h + childNode.g + childNode.potentialConflictsCount == node.h + node.g + node.potentialConflictsCount)
                            {
                                if (childNode.h == 0)
                                {
                                    this.openList.Add(childNode);
                                    return true;
                                }
                                this.expanded++;
                                if (Expand(childNode))
                                    return true;
                            }
                            else
                                this.openList.Add(childNode);

                            //if (childNode != null &&
                            //    childNode.allAgentsState[0].pos_X == 1 && childNode.allAgentsState[0].pos_Y == 0 &&
                            //    childNode.allAgentsState[1].pos_X == 2 && childNode.allAgentsState[1].pos_Y == 0 &&
                            //    childNode.allAgentsState[2].pos_X == 0 && childNode.allAgentsState[2].pos_Y == 2 &&
                            //    childNode.allAgentsState[3].pos_X == 1 && childNode.allAgentsState[3].pos_Y == 1)
                            //    Console.ReadLine();
                        }
                    }
                }
            }
            return false;
        }

        private void addToClosedList(WorldStateWithOD toAdd)
        {
            WorldStateWithOD cpy = new WorldStateWithOD(toAdd);
            cpy.mirrorState = toAdd;
            bool restricting;
            bool shouldAdd = false;
            for (int i = 0; i < cpy.agentTurn; i++)
            {
                restricting = false;
                for (int j = cpy.agentTurn; j < cpy.allAgentsState.Length && restricting == false ; j++)
                {
                    if (cpy.allAgentsState[i].pos_X == cpy.allAgentsState[j].pos_X && 
                        cpy.allAgentsState[i].pos_Y == cpy.allAgentsState[j].pos_Y)
                    {
                        restricting = true;
                    }
                }
                if (restricting == false)
                {
                    cpy.allAgentsState[i].direction = (int)Move.Direction.NO_DIRECTION;
                    shouldAdd = true;
                }
            }
            if (toAdd.agentTurn == 0 || shouldAdd)
            {
                closedList.Add(cpy,cpy);
            }
        }

        /// <summary>
        /// Check if a move is valid. This checks against:
        /// - Hitting a wall or an obstacle
        /// - Colliding with other agents
        /// </summary>
        /// <param name="aMove"></param>
        /// <param name="agentIndex">The index of the agent that wants to perform the move</param>
        /// <param name="node"></param>
        /// <returns>true if the move is valid</returns>
        protected bool IsValid(TimedMove aMove, int agentIndex, WorldStateWithOD node)
        {
            if (this.instance.IsValid(aMove) == false)
                return false;

            // Check against all the agents that have already moved to see if current move collides with their move
            for (int i = 0 ; i < agentIndex ; i++)
                if (aMove.isColliding(node.allAgentsState[i].pos_X,
                    node.allAgentsState[i].pos_Y,
                    node.allAgentsState[i].direction))
                    return false;
               
            return !IsMoveReserved(aMove);            
        }

        /// <summary>
        /// Check if the proposed move is reserved in the plan of another agent.
        /// This is used in Trevor's IndependenceDetection.
        /// </summary>
        /// <param name="aMove"></param>
        /// <returns>True if the move is reserved</returns>
        protected bool IsMoveReserved(TimedMove aMove)
        {
            if (this.illegalMoves != null)
            {
                return aMove.isColliding(this.illegalMoves);
            }
            return false;
        }

        /// <summary>
        /// Returns the found plan, or null if no plan was found.
        /// The returned plan contains only full states (and not the intermediate states).
        /// </summary>
        /// <returns></returns>
        public override Plan GetPlan()
        {
            return new Plan(((WorldStateWithOD)(this.GetGoal())));
        }

    }



}
