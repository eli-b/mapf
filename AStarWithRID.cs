using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CPF_experiment
{
    /// <summary>
    /// This Solver runs A* but from time to time tries to solve completely the expanded node, by using Standley's independent detection.
    /// </summary>
    class AStarWithRID : AStarWithOD
    {

        override public string GetName() { return "A*+RID"; }

        /// <summary>
        /// Expand a given node. This includes:
        /// - Generating all possible children
        /// - Inserting them to OPEN
        /// - Insert the generated nodes to the hashtable of nodes, currently implmented together with the closed list.
        /// </summary>
        /// <param name="parent"></param>
        


    }
}
