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
    mean_memory: str
    replans_count_mean: float
    replans_max_agents: int
    max_steps_window: int
    max_reached_window: int
    max_expanded_window: int
    n_livelock: int
    scen_succeed_count: int


def clean_data(episodes_df: pd.DataFrame):
    def clean_row(row):
        if any(
                [
                    row["train_time"] < 0,
                    row["exec_time"] < 0,
                    row["steps"] > 10000,
                    row["replans_max_size"] > row["n_agents"],
                    row["end_reason"] == "unknown_accross_episodes_rate",
                    row["end_reason"] == "unknown",
                    row["memory"] < 0,
                    row["end_reason"] == "success" and row["reward"] == 0,  # Had to do some steps for success.
                    row["reward"] > 0,  # Reward is always 0 during experiments.
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
        valid_df = instance_df[~instance_df["end_reason"].str.startswith("cleaned")]

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
        res["error_rate"] = len(instance_df[instance_df["end_reason"].str.startswith("cleaned")]) / len(instance_df)
        res["max_memory"] = np.max(valid_df["memory"])
        res["mean_memory"] = np.mean(valid_df["memory"])
        res["replans_count_mean"] = round(np.mean(valid_df["replans_count"]), 1)
        res["replans_max_agents"] = np.max(valid_df["replans_max_size"])
        res["max_steps_window"] = np.max(valid_df["max_steps_window"])
        res["max_reached_window"] = np.max(valid_df["max_reached_window"])
        res["max_expanded_window"] = np.max(valid_df["max_expanded_window"])
        res["n_livelock"] = round(np.mean(valid_df["n_livelock"]), 1)

        scen_succeed_df = instance_df.groupby("scen_id").agg(
            succeed=pd.NamedAgg(column="end_reason", aggfunc=lambda reasons: np.isin("success", reasons)))
        res["scen_succeed_count"] = len(scen_succeed_df[scen_succeed_df["succeed"] == True])

        # Rate Formatting
        rate_cols = [key for key in res.keys() if "rate" in key]
        for rate_col in rate_cols:
            res[rate_col] = pretty_rate(res[rate_col])

        # Time Formatting
        res["online_time"] = pretty_time(res["online_time"])
        res["offline_time"] = pretty_time(res["offline_time"])

        # Memory Formatting
        res["max_memory"] = pretty_memory(res["max_memory"])
        res["mean_memory"] = pretty_memory(res["mean_memory"])

        all_res.append(ProblemResult(**res))

    return pd.DataFrame(data=all_res)
