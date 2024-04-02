#!/bin/bash

# You might need to make this script executable by first running
# `chmod +x run.sh` in your terminal.

# Defaults
NUM_CORES=1
SIM_MODE=""
SIM_TYPE=""

L2_PREFETCHER=""
# Benchmark program
BENCHMARK=''

# Benchmark class/size
CLASS=''

# Build the gem5 simulator using the scons command
BUILD_FLAG=false
DEBUG_FLAG=false

GEM5_PATH_DEBUG="build/X86_MESI_Two_Level_Pythia/gem5.debug"
GEM5_PATH_OPT="build/X86/gem5.opt"
GEM5_PATH_FAST="build/X86_MESI_Two_Level/gem5.fast"

# Construct command
COMMAND=""
LOG_FILE=""

# Valid options
declare -A VALID_SIM_MODES=(["se"]=1 ["fs"]=1)
declare -A VALID_SIM_TYPES=(["debug"]=1 ["opt"]=1 ["fast"]=1)
declare -A VALID_BENCHMARKS=(["bt"]=1 ["cg"]=1 ["ep"]=1 ["ft"]=1 ["is"]=1 ["lu"]=1 ["mg"]=1 ["sp"]=1)
declare -A VALID_CLASSES=(["a"]=1 ["b"]=1 ["c"]=1)
declare -A VALID_L2_PREFETCHERS=(["Stride"]=1 ["Scooby"]=1 ["Tagged"]=1 ["SPP"]=1 ["IM"]=1 ["DCPT"]=1)

# Helper function to check validity
is_valid() {
    local -n arr=$1  # Create a reference to the array
    [[ -n "${arr[$2]}" ]]
}

show_usage() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -m | --sim_mode   Simulation mode (se or fs)"
    echo "  -t | --sim_type   Simulation type (debug, opt, fast)"
    echo "  -n | --num_cores  Number of cores"
    echo "  -b | --benchmark  Benchmark name"
    echo "  -c | --class      Benchmark class (a, b, c)"
    echo "  -d | --debug      Enable debug flag (0 or 1)"
    echo "  -h | --help       Display this help message"
    echo "Example:"
    echo "  $0 --build --sim_type fast"
    echo "  $0 --sim_mode se --sim_type fast --num_cores 2 --l2_pref SPP"
    echo "  $0 --sim_mode fs --sim_type fast --num_cores 2 --l2_pref SPP --benchmark cg --class a"
    echo "  $0 -m fs -t fast -n 2 -b cg -c a -l2p SPP"
    echo "  $0 -m fs -t fast -n 2 -b cg -c a -l2p Scooby"
    echo "  $0 -m fs -t fast -n 2 -b cg -c a -l2p DCPT"
    echo "  $0 -m se -t fast -n 8 -b cg -c a -l2p Scooby"
    echo "  $0 -m fs -t fast -n 8 -b cg -c a -l2p Scooby"
    echo "  $0 -m fs -t fast -n 4 -b cg -c a -l2p Scooby"
}

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -bd |--build)       BUILD_FLAG=true                     ;;
        -d  |--debug)       DEBUG_FLAG=true                     ;;
        -m  |--sim_mode)    SIM_MODE="$2";              shift   ;;
        -t  |--sim_type)    SIM_TYPE="$2";              shift   ;;
        -n  |--num_cores)   NUM_CORES="$2";             shift   ;;
        -l2p|--l2_pref)     L2_PREFETCHER="$2";         shift   ;;
        -b  |--benchmark)   BENCHMARK="$2";             shift   ;;
        -c  |--class)       CLASS="$2";                 shift   ;;
        -h  |--help)        show_usage;                 exit 0  ;;
        *)                  echo "Unknown option: $1";  show_usage; exit 1 ;;
    esac
    shift # Move to the next argument
done

if $BUILD_FLAG; then
    echo "Building gem5 with ${SIM_TYPE} option."
    if [[ $SIM_TYPE == "debug" ]]; then
        scons $GEM5_PATH_DEBUG | tee -i ./build_${SIM_TYPE}.log
        exit 0
    elif [[ $SIM_TYPE == "opt" ]]; then
        scons $GEM5_PATH_OPT | tee -i ./build_${SIM_TYPE}.log
        exit 0
    elif [[ $SIM_TYPE == "fast" ]]; then
        scons $GEM5_PATH_FAST | tee -i ./build_${SIM_TYPE}.log
        exit 0
    else
        echo "Invalid simulation type. Valid options are: ${!VALID_SIM_TYPES[@]}"
        exit 1
    fi
fi

echo "Simulation Mode:      $SIM_MODE"
echo "Simulator Type:       $SIM_TYPE"
echo "Number of Cores:      $NUM_CORES"
echo "L2 Cache Prefetcher:  $L2_PREFETCHER"
echo "Benchmark Program:    $BENCHMARK"
echo "Benchmark Class:      $CLASS"

# Validation
if ! is_valid VALID_SIM_MODES "$SIM_MODE"; then
    echo "Invalid simulation mode. Valid options are: ${!VALID_SIM_MODES[@]}"
    exit 1
fi

if ! is_valid VALID_SIM_TYPES "$SIM_TYPE"; then
    echo "Invalid simulator type. Valid options are: ${!VALID_SIM_TYPES[@]}"
    exit 1
fi

if ! is_valid VALID_L2_PREFETCHERS "$L2_PREFETCHER"; then
    echo "Invalid L2 prefetcher. Valid options are: ${!VALID_L2_PREFETCHERS[@]}"
    exit 1
fi

if [[ $SIM_MODE == "fs" ]]; then
    if ! is_valid VALID_BENCHMARKS "$BENCHMARK"; then
        echo "Invalid benchmark. Valid options are: ${!VALID_BENCHMARKS[@]}"
        exit 1
    fi

    if ! is_valid VALID_CLASSES "$CLASS"; then
        echo "Invalid class. Valid options are: ${!VALID_CLASSES[@]}"
        exit 1
    fi
fi

if ! [[ "$NUM_CORES" =~ ^[0-9]+$ ]]; then
    echo "Invalid number of cores. Must be an integer."
    exit 1
fi

# Get the current date for folder naming in MM-DD-YYYY format
date_stamp=$(date +%m-%d-%Y)

# Get the current timestamp for file naming in HH-MM 24-hour format
time_stamp=$(date +%H-%M)

WORKLOAD="npb-$BENCHMARK-$CLASS"

OUT_DIR_PREFIX="m5out_${date_stamp}_l2$L2_PREFETCHER"
OUT_DIR=""
SUB_OUT_DIR="${time_stamp}"

# Prefetcher's debug flag argument
PREF_DEBUG_FLAG="--debug-flags=HWPrefetchScooby"

CONFIG_FILE_SE="configs/multicore_gem5/multicore_with_classic_caches/se_top.py"
CONFIG_FILE_FS="configs/multicore_gem5/multicore_with_classic_caches/x86-npb-benchmarks-classic.py"

if [[ $SIM_TYPE == "debug" || $SIM_TYPE == "opt" ]]; then
    COMMAND="$COMMAND $GEM5_PATH_DEBUG"
    if $DEBUG_FLAG; then
        COMMAND="$COMMAND $PREF_DEBUG_FLAG"
    fi
elif [[ $SIM_TYPE == "fast" ]]; then
    COMMAND="$COMMAND $GEM5_PATH_FAST"
else
    echo "Error - The simulator type you specified is invalid."
    exit 1
fi

if [[ $SIM_MODE == "se" ]]; then
    LOG_FILE=${SIM_MODE}_${SIM_TYPE}_${NUM_CORES}
    OUT_DIR=${OUT_DIR_PREFIX}_${LOG_FILE}
    COMMAND="$COMMAND --outdir=./${OUT_DIR}/${SUB_OUT_DIR} $CONFIG_FILE_SE"
elif [[ $SIM_MODE == "fs" ]]; then
    LOG_FILE=${SIM_MODE}_${SIM_TYPE}_${NUM_CORES}_${WORKLOAD}
    OUT_DIR=${OUT_DIR_PREFIX}_${LOG_FILE}
    COMMAND="$COMMAND --outdir=./${OUT_DIR}/${SUB_OUT_DIR} $CONFIG_FILE_FS --workload=$WORKLOAD"
else
    echo "Error - The simulation mode you specified is invalid."
    exit 1
fi

COMMAND="$COMMAND --num-cores=$NUM_CORES --l2pf=$L2_PREFETCHER"

mkdir -p ${OUT_DIR}/${SUB_OUT_DIR}

echo "Resulting command to be executed: $COMMAND"

$COMMAND | tee -i ./${OUT_DIR}/${SUB_OUT_DIR}/run_${OUT_DIR}.log