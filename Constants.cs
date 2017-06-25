
namespace CPF_experiment
{
    class Constants
    {
        public static int MAX_TIME = 300000;

        /// <summary>
        /// The cost that is set to the algorithms when all the states have been expanded and no goal was found 
        /// </summary>
        public const int NO_SOLUTION_COST = -1;
        /// <summary>
        /// The cost that is set to the algorithms when they are halted due to out of time 
        /// </summary>
        public const int TIMEOUT_COST = -2;
        /// <summary>
        /// The cost that is set to the algorithms when they are halted due to out of memory
        /// </summary>
        public const int MAX_MEMORY_COST = -3; // TODO: Implement this, it's a great idea!
        /// <summary>
        /// The number of generated nodes after which a debug print will be given
        /// </summary>
        public const int GENERATED_PER_PRINT = 10000;
        /// <summary>
        /// Prime numbers for hashing
        /// </summary>
        public static readonly int[] PRIMES_FOR_HASHING = { 1, 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 71, 73, 79 }; //22 numbers - 1 isn't prime!
        /// <summary>
        /// Max Consecutive failures before an algorithm is omitted from the list of tested algorithms
        /// </summary>
        public const int MAX_FAIL_COUNT = 40;
        /// <summary>
        /// This determines whether the ICTS should search a solution with lowest conflicts for the ID framework
        /// </summary>
        public static bool EXHAUSTIVE_ICTS = false;
        /// <summary>
        /// Allow head-on collisions
        /// </summary>
        public const bool ALLOW_HEAD_ON_COLLISION = true;

        public const bool ALLOW_DIAGONAL_MOVE = false;
        public static readonly int NUM_ALLOWED_DIRECTIONS = ALLOW_DIAGONAL_MOVE ? Move.NUM_DIRECTIONS : Move.NUM_NON_DIAG_MOVES;

        public enum ProblemVariant : byte
        {
            ORIG = 0, // Moving from goal incurs cost equal to all waits in the goal, plus the movement out.
            NEW, // Waiting at the goal is free
        }
        /// <summary>
        /// For partial expansion
        /// </summary>
        public static ProblemVariant Variant = ProblemVariant.ORIG;
    }
}
