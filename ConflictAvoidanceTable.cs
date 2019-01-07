using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace CPF_experiment
{
    class ConflictAvoidanceTable : IReadOnlyDictionary<TimedMove, List<int>>
    {
        Dictionary<TimedMove, List<int>> timedMovesToAgentNumList = new Dictionary<TimedMove,List<int>>();
        Dictionary<Move, (int time, int agentNum)> atGoalWaitsToTimeAndAgentNum = new Dictionary<Move, (int time, int agentNum)>(); // No need for a list of agent nums because goals can't collide :)

        public void AddPlan(SinglePlan plan)
        {
            int planSize = plan.GetSize();
            for (int i = 0; i < planSize; i++)
            {
                Move temp = plan.GetLocationAt(i);
                TimedMove step;
                if (temp.GetType() == typeof(TimedMove))
                    step = (TimedMove) temp;
                else
                    step = new TimedMove(temp, i); // TODO: Avoid creating new objects when possible. Make the method return correctly timed moves.
                if (this.timedMovesToAgentNumList.ContainsKey(step) == false)
                    this.timedMovesToAgentNumList[step] = new List<int>(1); // THIS IS ON THE HOT PATH! ~11% of time is passed on this line!
                this.timedMovesToAgentNumList[step].Add(plan.agentNum);
            }

            Move lastMove = plan.GetLocationAt(planSize - 1);
            Move goal = new Move(lastMove.x, lastMove.y, Move.Direction.Wait);
            this.atGoalWaitsToTimeAndAgentNum.Add(goal, (planSize, plan.agentNum));
        }

        /// <summary>
        /// Gets an enumerable collection that contains the keys in the read-only dictionary
        /// </summary>
        /// <returns>An enumerable collection that contains the keys in the read-only dictionary</returns>
        public IEnumerable<TimedMove> Keys
        {
            get
            {
                throw new NotImplementedException(); // How would I know how many waits in the goal to return?
            }
        }

        /// <summary>
        /// Gets an enumerable collection that contains the values in the read-only dictionary
        /// </summary>
        /// <returns>An enumerable collection that contains the values in the read-only dictionary</returns>
        public IEnumerable<List<int>> Values
        {
            get
            {
                foreach (List<int> agentNumList in timedMovesToAgentNumList.Values)
                {
                    yield return agentNumList;
                }
            }
        }

        /// <summary>
        /// Gets the element that has the specified key in the read-only dictionary.
        /// </summary>
        /// <param name="key">The key to locate</param>
        /// <returns>The element that has the specified key in the read-only dictionary</returns>
        /// <exception cref="System.ArgumentNullException">key is null</exception>
        /// <exception cref="System.Collections.Generic.KeyNotFoundException">The property is retrieved and key is not found</exception>
        public List<int> this[TimedMove key]
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
                            ans = new List<int>(1);
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
        public bool TryGetValue(TimedMove key, out List<int> value)
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

        public IEnumerator<KeyValuePair<TimedMove, List<int>>> GetEnumerator()
        {
            throw new NotImplementedException(); // Don't know how many waits at the goal to return
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            throw new NotImplementedException(); // Don't know how many waits at the goal to return
        }

        public int Count
        {
            get
            {
                throw new NotImplementedException(); // Don't know how many waits at the goal we contain
            }
        }
    }
}
