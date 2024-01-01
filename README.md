# Unravelinator

Behold, the unravel-inator!

I'm currently porting this to OpenUSD from a Maya script I had written a while ago. Right now it unravels a torus with a hard-coded traversal order. But with some extra work it should be able to support any mesh you throw at it.

This script requires numpy, scipy, and the OpenUSD python bindings. You can get these with pip:

```
pip install numpy scipy usd-core
```

To generate the output `out.usda` file:

```
python3 torus_unravel.py
```

Some goals:
- Allow the user to provide any .usd file with a mesh in it
- Some timing options (i.e. how fast), forwards or backwards
- Support branches, multiple roots
- Allow the user to define unravel boundaries (probably through seams in a UV map?)
