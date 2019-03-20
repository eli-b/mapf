using System.Collections.Generic;
using System.Diagnostics;

namespace CPF_experiment
{
    public class CbsCacheEntry
    {
        protected CbsNode cbsNode;
        protected int agentIndex;

        public CbsCacheEntry(CbsNode cbsNode, int agentIndex)
        {
            this.cbsNode = cbsNode;
            this.agentIndex = agentIndex;
            Debug.Assert(cbsNode.cbs.mergeThreshold == -1, "When agents are merged it affects their paths without explicit constraints");
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int ans = 0;
                HashSet<CbsConstraint> constraints = this.cbsNode.GetConstraints();

                // Add the hash codes for the contraints, ignoring their order
                foreach (CbsConstraint constraint in constraints)
                {
                    if (constraint.agentNum == this.agentIndex)
                        ans += constraint.GetHashCode();
                }

                return ans;
            }
        }

        public override bool Equals(object obj)
        {
            if (obj == null)
                return false;
            CbsCacheEntry other = (CbsCacheEntry)obj;
            if (this.agentIndex != other.agentIndex)
                return false;
            HashSet<CbsConstraint> constraints = this.cbsNode.GetConstraints();
            constraints.RemoveWhere(constraint => constraint.agentNum != this.agentIndex);
            HashSet<CbsConstraint> otherConstraints = other.cbsNode.GetConstraints();
            otherConstraints.RemoveWhere(constraint => constraint.agentNum != other.agentIndex);
            return constraints.SetEquals(otherConstraints);
        }
    }
}
