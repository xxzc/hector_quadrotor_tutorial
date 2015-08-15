
class Inventory:
    """
    >>> i = Inventory()
    >>> i.add('a')
    >>> i.add('b')
    >>> i.remove('b')
    >>> print i
    a: 1
    >>> 'b' in i
    False
    """
    def __init__(self):
        self.all = {}

    def add(self, thing):
        assert type(thing) == str
        self.all[thing] = self.all.setdefault(thing, 0) + 1

    def remove(self, thing):
        if thing in self.all:
            self.all[thing] -= 1
            if self.all[thing] == 0:
                self.all.pop(thing)
        else:
            raise AssertionError('Remove from Empty.')

    def __contains__(self, key):
        return key in self.all

    def __str__(self):
        return '\n'.join(map(lambda i: '%s: %d'%i, self.all.items()))


class Station:
    def __init__(self, invent, pos, camsub):
        self.invent = invent
        self.pos = pos
        self.camsub = camsub

if __name__ == "__main__":
    import doctest
    doctest.testmod()
