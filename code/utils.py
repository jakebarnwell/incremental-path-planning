# -*- coding: utf-8 -*-

"""
Parts of the code in this file may be modified and/or original code by Peter Norvig.

Original Software License Agreement

Copyright © 1998-2002 by Peter Norvig.
Permission is granted to anyone to use this software, in source or object code form, on any computer system, and to modify, compile, decompile, run, and redistribute it to anyone else, subject to the following restrictions:

The author makes no warranty of any kind, either expressed or implied, about the suitability of this software for any purpose.
The author accepts no liability of any kind for damages or other consequences of the use of this software, even if they arise from defects in the software.
The origin of this software must not be misrepresented, either by explicit claim or by omission.
Altered versions must be plainly marked as such, and must not be misrepresented as being the original software. Altered versions may be distributed in packages under other licenses (such as the GNU license).
If you find this software useful, it would be nice if you let me (peter@norvig.com) know about it, and nicer still if you send me modifications that you are willing to share. However, you are not required to do so.

http://www.norvig.com/license.html
"""

def test_ok():
    try:
        from IPython.display import display_html
        display_html("""<div class="alert alert-success">
        <strong>Test passed!!</strong>
        </div>""", raw=True)
    except:
        print "test ok!!"


def update(x, **entries):
    """Update a dict; or an object with slots; according to entries.
    >>> update({'a': 1}, a=10, b=20)
    {'a': 10, 'b': 20}
    >>> update(Struct(a=1), a=10, b=20)
    Struct(a=10, b=20)
    """
    if isinstance(x, dict):
        x.update(entries)
    else:
        x.__dict__.update(entries)
    return x

def some(predicate, seq):
    """If some element x of seq satisfies predicate(x), return predicate(x).
    >>> some(callable, [min, 3])
    1
    >>> some(callable, [2, 3])
    0
    """
    for x in seq:
        px = predicate(x)
        if px: return px
    return False
