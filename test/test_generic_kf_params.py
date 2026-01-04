import pytest
from sensor_fusion_pkg.nodes.generic_kf_node import validate_dim_and_topics

def print_test_header(title):
    print(f"\n{'='*60}")
    print(f" TEST CASE: {title}")
    print(f"{'='*60}")

def test_validate_dim_and_topics_accepts_matching_length():
    print_test_header("Normal Case (Matching Dimensions)")
    
    dim_z = 2
    topics = ['/sensor_1/data', '/sensor_2/data']
    
    print(f"[ Input ]")
    print(f"  dim_z : {dim_z}")
    print(f"  topics: {topics}")
    
    # 実行（例外が出なければ成功）
    validate_dim_and_topics(dim_z, topics)
    
    print(f"\n[ Result ] OK (No exception raised)")


def test_validate_dim_and_topics_raises_on_length_mismatch():
    print_test_header("Error Case (Length Mismatch)")
    
    dim_z = 2
    topics = ['/sensor_1/data'] # 1つ足りない
    
    print(f"[ Input ]")
    print(f"  dim_z : {dim_z}")
    print(f"  topics: {topics} (len={len(topics)})")
    print(f"\n[ Expectation ] Should raise ValueError")

    # 例外の発生を確認
    with pytest.raises(ValueError) as excinfo:
        validate_dim_and_topics(dim_z, topics)
    
    # ここに来れば例外が正しく捕捉されている
    print(f"[ Result ] OK (Caught expected exception: {excinfo.type.__name__})")
    print(f"  -> Message: {excinfo.value}")


def test_validate_dim_and_topics_raises_on_non_positive_dim_z():
    print_test_header("Error Case (Non-positive dim_z)")

    # Case 1: 0
    dim_z_zero = 0
    topics = ['/sensor_1/data']
    print(f"\n--- Subcase: dim_z = 0 ---")
    print(f"  Input: dim_z={dim_z_zero}, topics={topics}")
    
    with pytest.raises(ValueError) as excinfo:
        validate_dim_and_topics(dim_z_zero, topics)
    print(f"  [ Result ] OK (Caught: {excinfo.value})")

    # Case 2: -1
    dim_z_neg = -1
    print(f"\n--- Subcase: dim_z = -1 ---")
    print(f"  Input: dim_z={dim_z_neg}, topics={topics}")
    
    with pytest.raises(ValueError) as excinfo:
        validate_dim_and_topics(dim_z_neg, topics)
    print(f"  [ Result ] OK (Caught: {excinfo.value})")


def test_validate_dim_and_topics_raises_on_non_list_topics():
    print_test_header("Error Case (Invalid Type for topics)")
    
    dim_z = 1
    bad_topics = '/sensor_1/data'  # listではなくstrを渡すミス
    
    print(f"[ Input ]")
    print(f"  dim_z : {dim_z}")
    print(f"  topics: {bad_topics} (type={type(bad_topics)})")
    print(f"\n[ Expectation ] Should raise ValueError")

    with pytest.raises(ValueError) as excinfo:
        validate_dim_and_topics(dim_z, bad_topics)
        
    print(f"[ Result ] OK (Caught expected exception: {excinfo.value})")
