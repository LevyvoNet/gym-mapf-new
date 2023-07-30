import pandas as pd
import os
import subprocess
import sys

import analysis

# SANITY_BENCHMARK_BINARY_PATH = f"{os.path.dirname(__file__)}/../sanity_benchmark"
# EPISODES_PATH = f"{os.path.dirname(__file__)}/../sanity_temp.log.csv"
SANITY_BENCHMARK_OUTPUT_FILE_NAME = "sanity_temp.log.csv"


def run_sanity_benchmark(sanity_benchmark_path: str):
    # Run the binary
    popen = subprocess.Popen(sanity_benchmark_path, stdout=subprocess.PIPE)
    popen.wait()

    episodes = pd.read_csv(os.path.join(os.getcwd(), SANITY_BENCHMARK_OUTPUT_FILE_NAME), index_col=False)
    episodes_agg = analysis.aggregate(episodes)
    episodes_agg = episodes_agg[[
        "map_name",
        "n_agents",
        "solver_name",
        "adr",
        "success_rate",
        "offline_time",
        "online_time",
        "max_memory",
        "replans_count_mean",
        "replans_max_agents",
    ]]

    print(episodes_agg.to_markdown())

    if analysis.any_failed(episodes):
        return 1

    return 0


if __name__ == "__main__":
    sanity_benchmark_path = sys.argv[1]
    sys.exit(run_sanity_benchmark(sanity_benchmark_path))
