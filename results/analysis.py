from dataclasses import dataclass
import numpy as np
import pandas as pd
import math


# TODO: check for collisions, there should not be an episode with a collision result.

def pretty_time(time_ms: int):
    MS_TO_S = 1 / 1000

    if math.isnan(time_ms):
        return -1

    return f"{round(time_ms * MS_TO_S)}s"


def pretty_memory(mem_bytes: int):
    MB_TO_GB = 1 / 10 ** 6

    if math.isnan(mem_bytes):
        return -1

    return f"{round(mem_bytes * MB_TO_GB)}GB"


def pretty_rate(rate: float) -> str:
    return f"{round(rate * 100, 1)}%"


@dataclass
class ProblemResult:
    """Class for aggregated result of a problem instance episodes.

    A problem instance is defined by its map, number of agents, scen id
    and solver.
    """
    map_name: str
    n_agents: int
    solver_name: str
    success_rate: str
    online_time: str
    adr: float
    adr_stderr: float
    offline_time: str
    timeout_rate: str
    stuck_rate: str
    exec_oom_rate: str
    train_oom_rate: str
    train_timeout_rate: str
    error_rate: str
    max_memory: str
    replans_count_mean: float
    replans_max_agents: int
    max_steps_window: int
    max_reached_window: int
    max_expanded_window: int
    n_livelock: int
    # scen_succeed: int


def clean_data(episodes_df: pd.DataFrame):
    def clean_row(row):
        if any(
                [
                    row["train_time"] < 0,
                    row["exec_time"] < 0,
                    row["reward"] > 0,
                    row["replans_max_size"] > row["n_agents"],
                    row["end_reason"] == "unknown_accross_episodes_rate",
                    row["end_reason"] == "unknown",
                ]):
            row["end_reason"] = f"cleaned ({row['end_reason']})"

        return row

    return episodes_df.apply(lambda row: clean_row(row), axis=1)


def aggregate(episodes_df: pd.DataFrame):
    episodes_df = clean_data(episodes_df)

    key_columns = ["map_name", "n_agents", "solver_name"]
    grouped_df = episodes_df.groupby(key_columns)
    all_res = []
    for group in grouped_df.groups:
        instance_df = grouped_df.get_group(group)
        success_df = instance_df[instance_df["end_reason"] == "success"]

        res = {}
        res["map_name"], res["n_agents"], res["solver_name"] = group
        res["success_rate"] = len(success_df) / len(instance_df)
        res["online_time"] = np.mean(success_df["exec_time"])
        res["adr"] = round(np.mean(success_df["reward"]), 1)
        res["adr_stderr"] = round(np.std(success_df["reward"]) / np.sqrt(np.size(success_df["reward"])), 1)
        res["offline_time"] = np.mean(success_df["train_time"])
        res["timeout_rate"] = len(instance_df[instance_df["end_reason"] == "timeout"]) / len(instance_df)
        res["stuck_rate"] = len(instance_df[instance_df["end_reason"] == "stuck"]) / len(instance_df)
        res["exec_oom_rate"] = len(instance_df[instance_df["end_reason"] == "exec_out_of_memory"]) / len(instance_df)
        res["train_oom_rate"] = len(instance_df[instance_df["end_reason"] == "train_out_of_memory"]) / len(instance_df)
        res["train_timeout_rate"] = len(instance_df[instance_df["end_reason"] == "train_timeout"]) / len(instance_df)
        res["max_memory"] = np.max(instance_df["memory"])
        res["replans_count_mean"] = np.mean(instance_df["replans_count"])
        res["replans_max_agents"] = np.max(instance_df["replans_max_size"])
        res["max_steps_window"] = np.max(instance_df["max_steps_window"])
        res["max_reached_window"] = np.max(instance_df["max_reached_window"])
        res["max_expanded_window"] = np.max(instance_df["max_expanded_window"])
        res["n_livelock"] = np.mean(instance_df["n_livelock"])
        res["error_rate"] = len(instance_df[instance_df["end_reason"].str.startswith("cleaned")]) / len(instance_df)

        # Rate Formatting
        rate_cols = [key for key in res.keys() if "rate" in key]
        for rate_col in rate_cols:
            res[rate_col] = pretty_rate(res[rate_col])

        # Time Formatting
        res["online_time"] = pretty_time(res["online_time"])
        res["offline_time"] = pretty_time(res["offline_time"])

        # Memory Formatting
        res["max_memory"] = pretty_memory(res["max_memory"])

        all_res.append(ProblemResult(**res))

    return pd.DataFrame(data=all_res)


PROBLEM_FAIL_STATUSES = [
    "train_out_of_memory",
    "train_timeout",
    "unknown_failure_across_episodes"
]


def any_failed(episodes):
    return all([
        episodes[episodes["end_reason"].isin(PROBLEM_FAIL_STATUSES)].empty,
        episodes[episodes["end_reason"].str.startswith("cleaned")].empty
    ])
