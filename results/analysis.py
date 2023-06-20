import pandas as pd
import numpy as np
import re

PROBLEM_FAIL_STATUSES = [
    "train_out_of_memory",
    "train_timeout",
    "unknown_failure_across_episodes"
]


def get_precent_number(rate: str):
    return int(re.search(r'\d+', rate).group())


def aggregate_results(episodes: pd.DataFrame):
    BYTES_TO_GB = 1 / 10 ** 9
    MS_TO_S = 1 / 1000
    END_REASON_SUCCESS = "success"
    SUCCESS_THRESHOLD = 97

    # NOTE: the scen_id is part of the group by only for sanity benchmark
    key_columns = ["map_name", "n_agents", "solver_name", "scen_id"]

    return episodes.groupby(key_columns).agg(
        ADR=pd.NamedAgg(column="reward", aggfunc=np.mean),
        ADR_STDERR=pd.NamedAgg(column="reward", aggfunc=lambda group: np.std(group) / np.sqrt(np.size(group))),
        rate=pd.NamedAgg(column="end_reason", aggfunc=lambda
            group: f"{int(len([e for e in group if e == END_REASON_SUCCESS]) / len(group) * 100)}%"),
        offline_time=pd.NamedAgg(column="train_time", aggfunc=lambda group: np.mean(group) * MS_TO_S),
        online_time=pd.NamedAgg(column="exec_time", aggfunc=lambda group: np.mean(group) * MS_TO_S),
        max_memory=pd.NamedAgg(column="memory", aggfunc=lambda group: np.max(group) * BYTES_TO_GB),
        mean_memory=pd.NamedAgg(column="memory", aggfunc=lambda group: np.mean(group) * BYTES_TO_GB),
        replans_count_mean=pd.NamedAgg(column="replans_count", aggfunc=np.mean),
        replans_max_agents=pd.NamedAgg(column="replans_max_size", aggfunc=np.mean),
    ).reset_index().groupby(["map_name", "n_agents", "solver_name"]).agg(
        ADR=pd.NamedAgg(column="ADR", aggfunc=np.mean),
        ADR_STDERR=pd.NamedAgg(column="ADR_STDERR", aggfunc=np.mean),
        offline_time=pd.NamedAgg(column="offline_time", aggfunc=np.mean),
        online_time=pd.NamedAgg(column="online_time", aggfunc=np.mean),
        max_memory=pd.NamedAgg(column="max_memory", aggfunc=np.max),
        mean_memory=pd.NamedAgg(column="mean_memory", aggfunc=np.mean),
        replans_count_mean=pd.NamedAgg(column="replans_count_mean", aggfunc=np.mean),
        replans_max_agents=pd.NamedAgg(column="replans_max_agents", aggfunc=np.max),
        success=pd.NamedAgg(column="rate", aggfunc=lambda rates: len(
            [r for r in rates if get_precent_number(r) >= SUCCESS_THRESHOLD])),
        rate=pd.NamedAgg(column="rate",
                         aggfunc=lambda rates: f"{int(np.mean([get_precent_number(r) for r in rates]))}%"),
    ).reset_index()


def any_failed(episodes):
    return not episodes[episodes["end_reason"].isin(PROBLEM_FAIL_STATUSES)].empty


def real_benchmark_agg(episodes: pd.DataFrame):
    IMPORTANT_COLUMNS = ["total_scen_count"]
    pass
