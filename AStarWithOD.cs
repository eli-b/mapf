using System;
using System.Collections.Generic;
using System.IO;

namespace CPF_experiment
{
    /// <summary>
    /// A* implementation with Standley's operator decomposition (OD). See AAAI 2010 paper by Trevor Scott Standley on Cooperative Pathfinding.
    /// </summary>
    public class AStarWithOD : ClassicAStar
    {
        public AStarWithOD(HeuristicCalculator heuristic = null)
            : base(heuristic) { }

        override protected WorldState CreateSearchRoot()
        {
            return new WorldStateWithOD(this.instance.m_vAgents);
        }

        override public string GetName() { return "A*+OD"; }

        /// <summary>
        /// Expand a given node. This includes:
        /// - Generating all possible children
        /// - Inserting them to OPEN
        /// - Insert the generated nodes to the hashtable of nodes, currently implmented together with the closed list.
        /// </summary>
        override public void Expand(WorldState node)
        {
            ExpandRecursively(node);
        }

        protected bool ExpandRecursively(WorldState node)
        {
            if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                return false;
            if (((WorldStateWithOD)node).agentTurn == 0)
                expandedFullStates++;
            //Debug.Print("Expanding node " + node);
            WorldStateWithOD parent = (WorldStateWithOD)node;
            int agentTurn = parent.agentTurn;
            WorldStateWithOD childNode;
            int childAgentTurn = ((parent.agentTurn + 1) % (this.instance.m_vAgents.Length));
            
            // Try all legal moves of the agents
            foreach (TimedMove newMove in parent.allAgentsState[agentTurn].last_move.GetNextMoves(Constants.ALLOW_DIAGONAL_MOVE))
            {
                if (this.IsValid(newMove, parent.currentMoves))
                {
                    childNode = new WorldStateWithOD(parent);
                    childNode.allAgentsState[agentTurn].move(newMove);
                    childNode.prevStep = parent;
                    childNode.agentTurn = childAgentTurn;
                    childNode.h = (int)this.heuristic.h(childNode);
                    // Makespan increases only if this is the move of the first agent
                    if (parent.agentTurn == 0)
                        childNode.makespan = parent.makespan + 1;
                    childNode.currentMoves.Add(newMove);

                    // g of child is equal to g of parent only when newMove is a STAY move and agent has already arrived at its goal
                    childNode.CalculateG();  

                    if (childNode.h + childNode.g <= this.maxCost)
                    {
                        if (instance.parameters.ContainsKey(Trevor.CONFLICT_AVOIDANCE))
                        {
                            childNode.potentialConflictsCount = parent.potentialConflictsCount;
                            childNode.potentialConflictsCount += childNode.conflictsCount(((HashSet<TimedMove>)instance.parameters[Trevor.CONFLICT_AVOIDANCE]));
                        }

                        if (instance.parameters.ContainsKey(CBS_LocalConflicts.INTERNAL_CAT))
                        {
                            childNode.cbsInternalConflictsCount = parent.prevStep.cbsInternalConflictsCount;
                            childNode.cbsInternalConflictsCount += parent.conflictsCount(((HashSet<TimedMove>)instance.parameters[CBS_LocalConflicts.INTERNAL_CAT]));
                        }

                        //if in closed list
                        if (this.closedList.ContainsKey(childNode) == true)
                        {
                            WorldStateWithOD inClosedList = (WorldStateWithOD)this.closedList[childNode];
                            if (inClosedList.mirrorState != null)
                                inClosedList = inClosedList.mirrorState; // Is mirror state guaranteed to also be in the closed list?
                            //if g is smaller than remove the old world state than remove state
                            if (inClosedList.g > childNode.g || 
                                (inClosedList.g == childNode.g && (inClosedList.potentialConflictsCount > childNode.potentialConflictsCount ||
                                (inClosedList.potentialConflictsCount == childNode.potentialConflictsCount && inClosedList.cbsInternalConflictsCount > childNode.cbsInternalConflictsCount))))
                            {
                                closedList.Remove(inClosedList); // Not removing the original, non-mirror state?
                                openList.Remove(inClosedList);
                            }
                        }

                        if (this.closedList.ContainsKey(childNode) == false)
                        {
                            this.generated++;
                            this.addToClosedList(childNode);
                            if (childNode.h + childNode.g + childNode.potentialConflictsCount == node.h + node.g + node.potentialConflictsCount)
                            {
                                if (childNode.h == 0)
                                {
                                    this.openList.Add(childNode);
                                    return true;
                                }
                                // Expanding the child node immediately - WHY???
                                this.expanded++;
                                if (ExpandRecursively(childNode))
                                    return true;
                            }
                            else
                                this.openList.Add(childNode);
                        }
                    }
                }
            }
            return false;
        }

        /// <summary>
        /// What's the logic behind this func?
        /// </summary>
        /// <param name="toAdd"></param>
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
                    if (cpy.allAgentsState[i].last_move.isColliding(cpy.allAgentsState[j].last_move)) // Behavior change: this didn't check for head-on collisions
                    {
                        restricting = true;
                        break;
                    }
                }
                if (restricting == false)
                {
                    cpy.allAgentsState[i].last_move.direction = Move.Direction.NO_DIRECTION; // ??
                    shouldAdd = true;
                }
            }
            if (shouldAdd || (toAdd.agentTurn == 0))
            {
                closedList.Add(cpy, cpy);
            }
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

        public override void OutputStatisticsHeader(TextWriter output)
        {
            base.OutputStatisticsHeader(output);

            output.Write(this.ToString() + " Expanded Full States (LL)");
            output.Write(Run.RESULTS_DELIMITER);
        }

        public override void OutputStatistics(TextWriter output)
        {
            base.OutputStatistics(output);

            Console.WriteLine("Total Expanded Full States (Low-Level): {0}", this.expandedFullStates);

            output.Write(this.expandedFullStates + Run.RESULTS_DELIMITER);
        }

        public int NumStatsColumns
        {
            get
            {
                return 1 + base.NumStatsColumns;
            }
        }
    }
}
