using System.Collections.Generic;
using System.Linq;

namespace mapf
{
    public class ConflictAvoidanceTable
    {
        Dictionary<TimedMove, List<int>> timedMovesToAgentNumList;
        Dictionary<Move, (int time, int agentNum)> atGoalWaitsToTimeAndAgentNum; // No need for a list of agent nums because goals can't collide :)

        public enum AvoidanceGoal
        {
            MINIMIZE_CONFLICTS,
            MINIMIZE_CONFLICTING_GROUPS
        };
        public AvoidanceGoal avoidanceGoal = AvoidanceGoal.MINIMIZE_CONFLICTS;

        public ConflictAvoidanceTable()
        {
            this.Clear();
        }

        public int GetMaxPlanSize()
        {
            if (atGoalWaitsToTimeAndAgentNum.Count > 0)
                return atGoalWaitsToTimeAndAgentNum.Values.Max(tuple => tuple.time) - 1;  // The first WAIT at goal we record is one time step after reaching it
            else
                return 0;
        }

        public void Clear()
        {
            timedMovesToAgentNumList = new Dictionary<TimedMove, List<int>>();
            atGoalWaitsToTimeAndAgentNum = new Dictionary<Move, (int time, int agentNum)>();
            NumPlans = 0;
        }
        
        public void AddPlan(SinglePlan plan)
        {
            int planSize = plan.GetSize();
            for (int i = 0; i < planSize; i++)
            {
                Move temp = plan.GetLocationAt(i);
                TimedMove step;
                if (temp.GetType() == typeof(TimedMove))
                    step = (TimedMove)temp;
                else  // It's a Move object
                {
                    queryTimedMove.setup(temp, i);
                    step = queryTimedMove;
                }
                if (this.timedMovesToAgentNumList.ContainsKey(step) == false)
                {
                    if (ReferenceEquals(step, queryTimedMove))  // Need a separate object that would serve as the key
                        step = new TimedMove(step);
                    this.timedMovesToAgentNumList[step] = new List<int>() { plan.agentNum };
                }
                else
                    this.timedMovesToAgentNumList[step].Add(plan.agentNum);
            }

            Move lastMove = plan.GetLocationAt(planSize - 1);
            var goal = new Move(lastMove.x, lastMove.y, Move.Direction.Wait);
            this.atGoalWaitsToTimeAndAgentNum[goal] = (planSize, plan.agentNum);
            ++NumPlans;
        }

        public void RemovePlan(SinglePlan plan)
        {
            int planSize = plan.GetSize();
            for (int i = 0; i < planSize; i++)
            {
                Move temp = plan.GetLocationAt(i);
                TimedMove step;
                if (temp.GetType() == typeof(TimedMove))
                    step = (TimedMove)temp;
                else  // It's a Move object
                {
                    queryTimedMove.setup(temp, i);
                    step = queryTimedMove;
                }
                this.timedMovesToAgentNumList[step].Remove(plan.agentNum);
                // TODO: Add asserts that check the plan was indeed in the CAT
            }

            Move lastMove = plan.GetLocationAt(planSize - 1);
            queryMove.setup(lastMove.x, lastMove.y, Move.Direction.Wait);
            this.atGoalWaitsToTimeAndAgentNum.Remove(queryMove);
            --NumPlans;
        }

        /// <summary>
        /// Gets the element that has the specified key in the read-only dictionary.
        /// </summary>
        /// <param name="key">The key to locate</param>
        /// <returns>The element that has the specified key in the read-only dictionary</returns>
        /// <exception cref="System.ArgumentNullException">key is null</exception>
        /// <exception cref="System.Collections.Generic.KeyNotFoundException">The property is retrieved and key is not found</exception>
        public IReadOnlyList<int> this[TimedMove key]
        {
            get
            {
                List<int> ans = null;
                if (this.timedMovesToAgentNumList.ContainsKey(key))
                {
                    ans = new List<int>(this.timedMovesToAgentNumList[key].Count + 1);
                    ans.AddRange(this.timedMovesToAgentNumList[key]);
                }
                
                queryMove.setup(key);
                if (this.atGoalWaitsToTimeAndAgentNum.ContainsKey(queryMove))
                {
                    var timeAndAgentNum = this.atGoalWaitsToTimeAndAgentNum[queryMove];
                    if (key.time >= timeAndAgentNum.time)
                    {
                        if (ans == null)
                            ans = new List<int>() { timeAndAgentNum.agentNum };
                        else
                            ans.Add(timeAndAgentNum.agentNum);
                    }
                }

                if (ans != null)
                    return ans;
                else
                    return ConflictAvoidanceTable.emptyList;
            }
        }

        private static readonly List<int> emptyList = new List<int>(0);

        private Move queryMove = new Move();
        private TimedMove queryTimedMove = new TimedMove();

        /// <summary>
        /// Determines whether the read-only dictionary contains an element that has
        ///  the specified key.
        /// </summary>
        /// <param name="key">The key to locate.</param>
        /// <returns>true if the read-only dictionary contains an element that has the specified key;
        /// otherwise, false.</returns>
        /// <exception cref="System.ArgumentNullException">key is null</exception>
        public bool ContainsKey(TimedMove key)
        {
            if (this.timedMovesToAgentNumList.ContainsKey(key))
                return true;

            queryMove.setup(key);
            if (this.atGoalWaitsToTimeAndAgentNum.ContainsKey(queryMove))
            {
                var timeAndAgentNum = this.atGoalWaitsToTimeAndAgentNum[queryMove];
                if (key.time >= timeAndAgentNum.time)
                    return true;
            }
            return false;
        }

        /// <summary>
        /// Gets the value that is associated with the specified key.
        /// </summary>
        /// <param name="key">The key to locate</param>
        /// <param name="value">
        /// When this method returns, the value associated with the specified key, if
        /// the key is found; otherwise, the default value for the type of the value
        /// parameter. This parameter is passed uninitialized.
        /// </param>
        /// <exception cref="System.ArgumentNullException">key is null</exception>
        /// <returns>
        /// true if the object that implements the System.Collections.Generic.IReadOnlyDictionary&lt;TKey,TValue&gt;
        /// interface contains an element that has the specified key; otherwise, false.
        /// </returns>
        public bool TryGetValue(TimedMove key, out IReadOnlyList<int> value)
        {
            if (this.ContainsKey(key))
            {
                value = this[key];
                return true;
            }
            else
            {
                value = null;
                return false;
            }
        }

        public int NumPlans { get; private set; }
    }
}
