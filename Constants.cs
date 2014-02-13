using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CPF_experiment
{
    class Constants
    {
        public static int MAX_TIME = 300000;

        // The cost that is set to the algorithms when all the states have been expanded and no goal was found
        public static int NO_SOLUTION_COST = -1;
        // The cost that is set to the algorithms when they are halted due to out of time
        public static int TIMEOUT_COST = -2;
        // The cost that is set to the algorithms when they are halted due to out of memory
        public static int MAX_MEMORY_COST = -3;
        // The number of generated nodes after which a debug print will be given
        public static int GENERATED_PER_PRINT = 10000;
        //prime numbers for hashing
        public static int[] PRIMES_FOR_HASHING = { 1, 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 71, 73, 79 };//22 numbers
        //group size for matching and pruning MDDs in a given ICTS node
        public static int MAX_FAIL_COUNT = 40;
        //this determins whether the ICTS should search a solution with lowest conflicts for th ID framework
        public static bool exausitveIcts = false;
    }
}
