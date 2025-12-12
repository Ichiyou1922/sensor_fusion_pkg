import pytest

from sensor_fusion_pkg.nodes.generic_kf_node import validate_dim_and_topics


def test_validate_dim_and_topics_accepts_matching_length():
    # dim_z とトピック数が一致しているので例外は出ないはず
    validate_dim_and_topics(2, ['/sensor_1/data', '/sensor_2/data'])


def test_validate_dim_and_topics_raises_on_length_mismatch():
    # dim_z=2 なのにトピックが1つ-> 例外が出なければならない
    with pytest.raises(ValueError):
        validate_dim_and_topics(2, ['/sensor_1/data'])


def test_validate_dim_and_topics_raises_on_non_positive_dim_z():
    with pytest.raises(ValueError):
        validate_dim_and_topics(0, ['/sensor_1/data'])

    with pytest.raises(ValueError):
        validate_dim_and_topics(-1, ['/sensor_1/data'])


def test_validate_dim_and_topics_raises_on_non_list_topics():
    # sensor_topics が list / tuple 以外なら弾く
    with pytest.raises(ValueError):
        validate_dim_and_topics(1, '/sensor_1/data')  # str を直接渡したケース
