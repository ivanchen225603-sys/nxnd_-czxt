"""指定原始双云子集只改变输入选择，不能按结果挑样本。"""

import pytest

from tools.replay_empty22_final_pcd import EXCLUDED_ID, selected_pairs


def manifest():
    return {'pairs': [{'id': str(index), 'inputUnit': 'm'} for index in range(22)]
                     + [{'id': EXCLUDED_ID, 'inputUnit': 'm'}]}


def test_default_selection_still_includes_all_22_valid_pairs():
    assert len(selected_pairs(manifest())) == 22


def test_explicit_selection_returns_exact_ids_in_canonical_order():
    assert [pair['id'] for pair in selected_pairs(manifest(), ['8', '2'])] == ['2', '8']


@pytest.mark.parametrize('ids', [[], ['1', '1'], ['unknown'], [EXCLUDED_ID]])
def test_invalid_subset_is_rejected(ids):
    with pytest.raises(ValueError):
        selected_pairs(manifest(), ids)
