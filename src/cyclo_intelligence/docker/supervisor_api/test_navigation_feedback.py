"""Keep diagnostic feedback available when a mission navigation fails."""

from supervisor_api.navigation import _navigate_diagnostic


def test_timeout_reports_last_feedback_not_initial_distance():
    feedback = '''distance_remaining: 3.0
number_of_recoveries: 0
distance_remaining: 0.053
number_of_recoveries: 2
'''
    assert _navigate_diagnostic(feedback, 'TIMEOUT', 120) == (
        'No terminal Nav2 result within 120 wall seconds; '
        'remaining distance=0.053 m; recoveries=2'
    )


def test_terminal_error_code_is_visible():
    assert _navigate_diagnostic('error_code: 42', 'ABORTED') == 'Nav2 error code=42'


def test_no_feedback_timeout_remains_useful():
    assert _navigate_diagnostic('Goal accepted', 'TIMEOUT', 240) == (
        'No terminal Nav2 result within 240 wall seconds'
    )
