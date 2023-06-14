import pandas as pd
import numpy as np

PROBLEM_FAIL_STATUSES = [
    "train_out_of_memory",
    "train_timeout",
    "unknown_failure_across_episodes"
]


def aggregate_results(episodes: pd.DataFrame):
    BYTES_TO_GB = 1 / 10 ** 9
    MS_TO_S = 1 / 1000
    END_REASON_SUCCESS = "success"

    # NOTE: the scen_id is part of the group by only for sanity benchmark
    key_columns = ["map_name", "n_agents", "solver_name", "scen_id"]

    # TODO: add reset_index() at the end. It's nicer.
    return episodes.groupby(key_columns).agg(
        ADR=pd.NamedAgg(column="reward", aggfunc=np.mean),
        ADR_STDERR=pd.NamedAgg(column="reward", aggfunc=lambda group: np.std(group) / np.sqrt(np.size(group))),
        rate=pd.NamedAgg(column="end_reason", aggfunc=lambda
            group: f"{len([e for e in group if e == END_REASON_SUCCESS]) / len(group) * 100}%"),
        offline_time=pd.NamedAgg(column="train_time", aggfunc=lambda group: np.mean(group) * MS_TO_S),
        online_time=pd.NamedAgg(column="exec_time", aggfunc=lambda group: np.mean(group) * MS_TO_S),
        max_memory=pd.NamedAgg(column="memory", aggfunc=lambda group: np.max(group) * BYTES_TO_GB),
        mean_memory=pd.NamedAgg(column="memory", aggfunc=lambda group: np.mean(group) * BYTES_TO_GB),
        replans_count_mean=pd.NamedAgg(column="replans_count", aggfunc=np.mean),
        replans_max_agents=pd.NamedAgg(column="replans_max_size", aggfunc=np.mean),
    ).reset_index()


def any_failed(episodes):
    return not episodes[episodes["end_reason"].isin(PROBLEM_FAIL_STATUSES)].empty


def real_benchmark_agg(episodes: pd.DataFrame):
    IMPORTANT_COLUMNS = ["total_scen_count"]
    pass
