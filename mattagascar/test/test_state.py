from mattagascar import iliketomoveitmoveit


def test_state():
    home_state = iliketomoveitmoveit.State.PLANHOME

    assert home_state == iliketomoveitmoveit.State.PLANHOME, 'Found Error with State!!'
