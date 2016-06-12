# Service generator

The `servicegen.pl` script generates code for kRPC services based on json files provided by the kRPC mod.

## Depedencies

* [JSON](https://metacpan.org/pod/JSON)
* [Template](https://metacpan.org/pod/Template) (template toolkit)

Those are popular modules and they are likely to be available in your distribution's packages.

## Usage

```bash
~/krpc-hs$ perl servicegen.pl /path/to/KSP/game/GameData/kRPC/KRPC.SpaceCenter.json > src/KRPCHS/SpaceCenter.hs
```

Here's a loop you can use to generate code for all services:

```bash
for f in `find /path/to/KSP/game/GameData/kRPC/ -name 'KRPC.*.json'`
do
	n=$(basename $f .json | sed 's/KRPC\.//')
	perl tools/servicegen.pl $f > src/KRPCHS/$n.hs
done
```

Don't forget to **add the new service to the .cabal file before you rebuild**.
