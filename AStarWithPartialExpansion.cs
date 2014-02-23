using System.Collections.Generic;
using System.IO;

namespace CPF_experiment
{
    class AStarWithPartialExpansionBasic : ClassicAStar
    {
        int generatedAndDiscarded;
        bool hasMoreSuc;

        public override void Setup(ProblemInstance problemInstance) 
        { 
            base.Setup(problemInstance);
            generatedAndDiscarded = 0;
        }

        override public string GetName() { return "PE-Basic "; }

        override public bool Expand(WorldState node)
        {
            //Debug.Print("Expanding node " + node);
            if (node.notExpanded)
            {
                node.notExpanded = false;
                this.expandedFullStates++;
            }

            hasMoreSuc = false;
            node.nextFvalue = byte.MaxValue;

            expand(node, 0, runner, node.h + node.g, new HashSet<TimedMove>());
            node.h = node.nextFvalue - node.g;

            if (hasMoreSuc && node.h + node.g <= this.maxCost)
                this.openList.Add(node);
            return true;
        }

        /// <summary>
        /// Another recursive implementation!
        /// </summary>
        /// <param name="currentNode"></param>
        /// <param name="agentIndex"></param>
        /// <param name="runner"></param>
        /// <param name="targetF"></param>
        /// <param name="currentMoves"></param>
        /// <returns></returns>
        protected bool expand(WorldState currentNode, int agentIndex, Run runner, int targetF, HashSet<TimedMove> currentMoves)
        {
            if (runner.ElapsedMilliseconds() > Constants.MAX_TIME || this.foundGoal)
                return true;
            WorldState prev = currentNode.prevStep;
            if (agentIndex == 0) // If this is the first agent that moves
            {
                hasMoreSuc = false;
                prev = currentNode;
                currentMoves.Clear();  
            }
            if (agentIndex == instance.m_vAgents.Length) // If all the agents have moved
            {

                currentNode.h = (int)this.heuristic.h(currentNode);
                currentNode.makespan++;
                currentNode.CalculateG();
                if (currentNode.h + currentNode.g <= this.maxCost)
                {
                    if (currentNode.h + currentNode.g == targetF)
                    {
                        if (instance.parameters.ContainsKey(Trevor.CONFLICT_AVOIDENCE))
                        {
                            currentNode.potentialConflictsCount = currentNode.prevStep.potentialConflictsCount;
                            currentNode.potentialConflictsCount += currentNode.conflictsCount(((HashSet<TimedMove>)instance.parameters[Trevor.CONFLICT_AVOIDENCE]));
                        }

                        if (instance.parameters.ContainsKey(CBS_LocalConflicts.INTERNAL_CAT))
                        {
                            currentNode.dncInternalConflictsCount = currentNode.prevStep.dncInternalConflictsCount;
                            currentNode.dncInternalConflictsCount = currentNode.conflictsCount(((HashSet<TimedMove>)instance.parameters[CBS_LocalConflicts.INTERNAL_CAT]));
                        }

                        //if in closed list
                        if (this.closedList.Contains(currentNode) == true)
                        {
                            WorldState inClosedList = (WorldState)this.closedList[currentNode];
                            //if g is smaller than remove the old world state
                            if (inClosedList.g > currentNode.g)
                            {
                                closedList.Remove(inClosedList);
                                openList.Remove(inClosedList);
                            }
                        }
                        if (this.closedList.Contains(currentNode) == false)
                        {

                            this.generated++;
                            this.closedList.Add(currentNode);


                            //if (currentNode.h + currentNode.g + currentNode.potentialConflictsCount == targetF + currentNode.prevStep.potentialConflictsCount)
                            //{
                            //    if (currentNode.h == 0)
                            //    {
                            //        this.openList.Add(currentNode);
                            //        this.foundGoal = true;
                            //        return true;
                            //    }
                            //    this.expanded++;
                            //    if (Expand(currentNode))
                            //        return true;
                            //}
                            //else
                                this.openList.Add(currentNode);
                            return true;
                        }
                    }
                    else
                        generatedAndDiscarded++;
                    if (currentNode.h + currentNode.g > targetF)
                    {
                        hasMoreSuc = true;
                        if (currentNode.h + currentNode.g < currentNode.prevStep.nextFvalue)
                            currentNode.prevStep.nextFvalue = (byte)(currentNode.h + currentNode.g);
                    }
                    return false;
                }
                return false;
            }

            // Try all legal moves of the agents
            TimedMove agentLocation = new TimedMove();
            DnCConstraint nextStepLocation = new DnCConstraint();
            int deltaX;
            int deltaY;
            int posX = currentNode.allAgentsState[agentIndex].pos_X;
            int posY = currentNode.allAgentsState[agentIndex].pos_Y;
            WorldState childNode;
            bool ans = false;
            for (int op = 0; op < WorldState.operators.GetLength(0); op++)
            {
                if (foundGoal)
                    return true;
                deltaX = WorldState.operators[op,0];
                deltaY = WorldState.operators[op,1];
                if (this.constraintList != null)
                {
                    nextStepLocation.init(instance.m_vAgents[agentIndex].agent.agentNum, posX + deltaX, posY + deltaY, currentNode.makespan + 1, WorldState.operators[op, 2]);
                    if (constraintList.Contains(nextStepLocation))
                        continue;
                }
                agentLocation.setup(posX + deltaX, posY + deltaY, WorldState.operators[op, 2], currentNode.makespan + 1);
                if (IsValid(agentLocation, currentMoves))
                {
                    currentMoves.Add(agentLocation);
                    childNode = new WorldState(currentNode);
                    childNode.allAgentsState[agentIndex].move(op);
                    childNode.prevStep = prev;
                    if (expand(childNode, agentIndex + 1, runner, targetF, currentMoves))
                        ans = true;
                    currentMoves.Remove(agentLocation);
                }
            }
            return ans;
        }

        public override void OutputStatistics(TextWriter output)
        {
            output.Write(this.expanded + Run.RESULTS_DELIMITER);
            output.Write(this.generated + Run.RESULTS_DELIMITER);
            output.Write("N/A" + Run.RESULTS_DELIMITER);
            output.Write(this.generatedAndDiscarded + Run.RESULTS_DELIMITER);
            output.Write(solutionDepth + Run.RESULTS_DELIMITER);
            output.Write(expandedFullStates + Run.RESULTS_DELIMITER);
            output.Write("NA"/*Process.GetCurrentProcess().VirtualMemorySize64*/ + Run.RESULTS_DELIMITER);
        }
    }
    

    class AStarWithPartialExpansion : ClassicAStar 
    {
        sbyte[][] fLookup;

        public override void Setup(ProblemInstance problemInstance)
        {
            this.instance = problemInstance;
            this.heuristic = this.CreateHeuristic();
            WorldState root = this.CreateSearchRoot();
            root.h = (int)this.heuristic.h(root);
            this.closedList.Add(root);
            this.openList.Add(root);
            this.expanded = 0;
            expandedFullStates = 0;
            this.totalCost = 0;
            this.solutionDepth = 0;
            this.goal = null;
            this.numOfAgents = problemInstance.m_vAgents.Length;
            this.generated = 0;

            // Store parameters used by Trevor's Independant Detection algorithm
            if (problemInstance.parameters.ContainsKey(Trevor.MAXIMUM_COST_KEY))
                this.maxCost = (int)(problemInstance.parameters[Trevor.MAXIMUM_COST_KEY]);
            else
                this.maxCost = int.MaxValue;
            if (problemInstance.parameters.ContainsKey(Trevor.ILLEGAL_MOVES_KEY))
                this.illegalMoves = (HashSet<TimedMove>)(problemInstance.parameters[Trevor.ILLEGAL_MOVES_KEY]);
            else
                this.illegalMoves = null;

        }

        override protected WorldState CreateSearchRoot()
        {
            WorldStateForPartialExpansion root = new WorldStateForPartialExpansion(this.instance.m_vAgents);
            //root.getNextChild(instance, closedList, true);
            return root;
        }

        override public string GetName() { return "PEA* "; }

        public override bool Expand(WorldState nodeP)
        {
            WorldStateForPartialExpansion node = (WorldStateForPartialExpansion)nodeP;
            fLookup = null;

            byte[][] allMoves = node.getSingleAgentMoves(instance);
            int maxFchange = node.getMaxFchange(allMoves);

            if (node.isAlreadyExpanded() == false)
            {
                expandedFullStates++;
                node.alreadyExpanded = true;
            }
            //Debug.Print("Expanding node " + node);


            sbyte[][] fLookupTable = null; // [0] - agent number ,[1] - f change, value =1 - exists successor, value = -1 not exists, value = 0 don't know

            Expand(node, 0, runner, new HashSet<TimedMove>(), allMoves, node.currentFChange, fLookupTable);
            node.currentFChange++;
            node.h++;
            while (node.hasMoreChildren(maxFchange) && existingChildForF(allMoves, 0, node.currentFChange) == false)
            {
                node.currentFChange++;
                node.h++;
            }

            if (node.hasMoreChildren(maxFchange) && existingChildForF(allMoves, 0, node.currentFChange) && node.h + node.g <= this.maxCost)
                openList.Add(node);
            return true;
        }

        protected bool Expand(WorldState currentNode, int agentIndex, Run runner, HashSet<TimedMove> currentMoves, byte[][] allMoves, int targetFchange, sbyte[][] fLookupTable)
        {
            if (existingChildForF(allMoves, agentIndex, targetFchange) == false)
                return false;

            if (targetFchange < 0)
                return false;
            if (runner.ElapsedMilliseconds() > Constants.MAX_TIME || this.foundGoal)
                return true;
            WorldStateForPartialExpansion prev = (WorldStateForPartialExpansion)currentNode.prevStep;
            if (agentIndex == 0) // If this is the first agent that moves
            {
                prev = (WorldStateForPartialExpansion)currentNode;
            }
            if (agentIndex == instance.m_vAgents.Length) // If all the agents have moved
            {
                if (targetFchange != 0)
                    return false;


                currentNode.h = (int)this.heuristic.h(currentNode);
                currentNode.makespan++;
                currentNode.CalculateG();
                if (currentNode.h + currentNode.g <= this.maxCost)
                {

                    if (instance.parameters.ContainsKey(Trevor.CONFLICT_AVOIDENCE))
                    {
                        currentNode.potentialConflictsCount = currentNode.prevStep.potentialConflictsCount;
                        currentNode.potentialConflictsCount += currentNode.conflictsCount(((HashSet<TimedMove>)instance.parameters[Trevor.CONFLICT_AVOIDENCE]));
                    }

                    if (instance.parameters.ContainsKey(CBS_LocalConflicts.INTERNAL_CAT))
                    {
                        currentNode.dncInternalConflictsCount = currentNode.prevStep.dncInternalConflictsCount;
                        currentNode.dncInternalConflictsCount += currentNode.conflictsCount(((HashSet_U<TimedMove>)instance.parameters[CBS_LocalConflicts.INTERNAL_CAT]));
                    }

                    //if in closed list
                    if (this.closedList.Contains(currentNode) == true)
                    {
                        WorldStateForPartialExpansion inClosedList = (WorldStateForPartialExpansion)this.closedList[currentNode];
                        //if g is smaller than remove the old world state
                        if (inClosedList.g > currentNode.g || (inClosedList.g == currentNode.g && (inClosedList.potentialConflictsCount > currentNode.potentialConflictsCount || (inClosedList.potentialConflictsCount == currentNode.potentialConflictsCount && inClosedList.dncInternalConflictsCount > currentNode.dncInternalConflictsCount))))
                        {
                            closedList.Remove(inClosedList);
                            openList.Remove(inClosedList);
                        }
                    }
                    if (this.closedList.Contains(currentNode) == false)
                    {
                        this.generated++;
                        this.closedList.Add(currentNode);

                        //if (currentNode.h + currentNode.g + currentNode.potentialConflictsCount == currentNode.prevStep.h + currentNode.prevStep.g + currentNode.prevStep.potentialConflictsCount)
                        //{
                        //    if (currentNode.h == 0)
                        //    {
                        //        this.foundGoal = true;
                        //        this.openList.Add(currentNode);
                        //    }
                        //    else
                        //    {
                        //        expanded++;
                        //        Expand(currentNode);
                        //    }
                        //}
                        //else
                            this.openList.Add(currentNode);
                        return true;
                    }
                }
                return false;
            }

            // Try all legal moves of the agents
            TimedMove agentLocation = new TimedMove();
            DnCConstraint nextStepLocation = new DnCConstraint();
            int deltaX;
            int deltaY;
            int posX = currentNode.allAgentsState[agentIndex].pos_X;
            int posY = currentNode.allAgentsState[agentIndex].pos_Y;
            WorldStateForPartialExpansion childNode;
            bool ans = false;
            for (int op = 0; op < 5; op++)
            {
                if (foundGoal)
                    return true;
                deltaX = WorldState.operators[op, 0];
                deltaY = WorldState.operators[op, 1];
                if (this.constraintList != null)
                {
                    nextStepLocation.init(instance.m_vAgents[agentIndex].agent.agentNum, posX + deltaX, posY + deltaY, currentNode.makespan + 1, WorldState.operators[op, 2]);
                    if (constraintList.Contains(nextStepLocation))
                        continue;
                }
                agentLocation.setup(posX + deltaX, posY + deltaY, WorldState.operators[op, 2], currentNode.makespan + 1);
                if (IsValid(agentLocation, currentMoves))
                {
                    currentMoves.Add(agentLocation);
                    childNode = new WorldStateForPartialExpansion(currentNode);
                    childNode.allAgentsState[agentIndex].move(op);
                    childNode.prevStep = prev;
                    if (Expand(childNode, agentIndex + 1, runner, currentMoves, allMoves, targetFchange - allMoves[agentIndex][op],fLookupTable))
                        ans = true;
                    currentMoves.Remove(agentLocation);
                }
            }
            return ans;
        }

        public bool existingChildForF(byte[][] allMoves, int agent, int targetFchange) 
        {
            // allMoves[][] = [0] - agent number [1] - direction [in table]- effecte on F)
            // fLookup[][] = [0] - agent number ,[1] - f change, value =1 - exists successor, value = -1 not exists, value = 0 dont know

            if (targetFchange < 0)
            {
                return false;
            }

            if (agent == allMoves.Length)
            {
                if (targetFchange == 0)
                    return true;
                return false;
            }

            if (fLookup == null)
            {
                fLookup = new sbyte[allMoves.Length][];
                for (int i = 0; i < fLookup.Length; i++)
                {
                    fLookup[i] = new sbyte[1 + 2 * fLookup.Length];
                }
            }

            if (targetFchange + 1 > fLookup[agent].Length)
            {
                sbyte[] old = fLookup[agent];
                fLookup[agent] = new sbyte[targetFchange + 1];
                for (int i = 0; i < old.Length; i++)
                {
                    fLookup[agent][i] = old[i];
                }
            }

            if (fLookup[agent][targetFchange] != 0)
            {
                return fLookup[agent][targetFchange] == 1; 
            }

            for (int i = 0; i < allMoves[agent].Length; i++)
            {
                if (allMoves[agent][i] > targetFchange)
                    continue;
                if (existingChildForF(allMoves, agent + 1, targetFchange - allMoves[agent][i]))
                {
                    fLookup[agent][targetFchange] = 1;
                    return true;
                }
            }
            fLookup[agent][targetFchange] = -1;
            return false;
        }
    }
}
