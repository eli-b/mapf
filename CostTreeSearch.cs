using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CPF_experiment
{
    class CostTreeSearch : Trevor
    {
        public static string PARENT_GROUP1_KEY = "parentGroup1";
        public static string PARENT_GROUP2_KEY = "parentGroup2";


        public CostTreeSearch()
            : base(new CostTreeSearchSolver()){}

        public CostTreeSearch(ISolver groupSolver)
            : base(groupSolver){}

        public override string GetName() { return "CostTreeSearch+ID "; }

        /// <summary>
        /// Join the conflicting groups into a single group
        /// </summary>
        /// <param name="conflict">An object that describes the conflict</param>
        /// <returns>The composite group of agents</returns>
        protected override AgentsGroup JoinGroups(Conflict conflict)
        {
            AgentsGroup answer = conflict.group1.Join(conflict.group2);
            // TODO: Currently storing the previous groups - this might lead to a memory problem when there are many agents
            // (if this happens then store only the costs)
            answer.instance.parameters[PARENT_GROUP1_KEY] = conflict.group1;
            answer.instance.parameters[PARENT_GROUP2_KEY] = conflict.group2;

            // Free memory of grandparents
            conflict.group1.instance.parameters.Remove(PARENT_GROUP1_KEY);
            conflict.group1.instance.parameters.Remove(PARENT_GROUP2_KEY);
            conflict.group2.instance.parameters.Remove(PARENT_GROUP1_KEY);
            conflict.group2.instance.parameters.Remove(PARENT_GROUP2_KEY);

            return answer;
        }

    }
    class CostTreeSearchOldMatching : CostTreeSearch
    {
        int sycSize;
        public CostTreeSearchOldMatching(int sycSize)
            : base(new CostTreeSearchSolverOldMatching(sycSize)) { this.sycSize = sycSize; }

        public override string GetName() { return "ICTS " + sycSize + "E+ID "; }
    }
    class CostTreeSearchNoPruning : CostTreeSearch
    {
        public CostTreeSearchNoPruning()
            : base(new CostTreeSearchSolverNoPruning()){}

        public override string GetName() { return "ICTS " + "+ID "; }

    }

    class CostTreeSearchKMatch : CostTreeSearch
    {
        int maxGroupChecked;
        public CostTreeSearchKMatch(int maxGroupChecked)
            : base(new CostTreeSearchSolverKMatch(maxGroupChecked)) { this.maxGroupChecked = maxGroupChecked; }


        public override string GetName() { return "ICTS " + maxGroupChecked + "S+ID "; }

    }

    class CostTreeSearchRepatedMatch : CostTreeSearch
    {
        int sycSize;
        public CostTreeSearchRepatedMatch(int sycSize)
            : base(new CostTreeSearchSolverRepatedMatch(sycSize)) { this.sycSize = sycSize; }

        public override string GetName() { return "ICTS " + sycSize + "RE+ID "; }
    }
}
