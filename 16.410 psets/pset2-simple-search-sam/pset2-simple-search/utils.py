import IPython
from nose.tools import assert_equal, ok_
from search_classes import SearchNode

def test_ok():
    try:
        from IPython.display import display_html
        display_html("""<div class="alert alert-success">
        <strong>Test passed!!</strong>
        </div>""", raw=True)
    except:
        print "test ok!!"


def check_expanded_states(returned_states, correct_states):
    ok_(isinstance(returned_states, list), msg="Your function should return a list.")
    assert_equal(len(returned_states), len(correct_states),
                 msg="Your function returned %d states, but it should have returned %d"%(len(returned_states),
                                                                                             len(correct_states)))
    for s in returned_states:
        ok_(isinstance(s, tuple),
            msg="Each state returned by your function should be a tuple. %s is not" %(s,))
        ok_(len(s)==3, msg="Each state should have three internal tuples, %s doesn't." %(s,))
        for l in s:
            ok_(isinstance(l, tuple),
            msg="Each state should have three internal tuples, %s doesn't." %(s,))
            ok_(len(l)==3, msg="Each internal tuple of a state should have three elements, %s doesn't" %(s,))

    for correct_state in correct_states:
        ok_(correct_state in returned_states, msg="%s state is not in returned states, and it should be."%(correct_state,))
    

def check_expanded_nodes(returned_nodes, parent_node, correct_states):
    ok_(isinstance(returned_nodes, list), msg="Your function should return a list.")
    assert_equal(len(returned_nodes), len(correct_states),
                msg="Your function returned %d nodes, but it should have returned %d"%(len(returned_nodes),
                len(correct_states)))

    for n in returned_nodes:
        ok_(isinstance(n, SearchNode), msg="Your function should return a list of SearchNodes.")
        ok_(n.parent is parent_node, msg="The parent node for node %s is wrong." % n)

    check_expanded_states([n.state for n in returned_nodes], correct_states)
        
