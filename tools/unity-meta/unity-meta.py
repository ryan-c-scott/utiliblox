import sys, os, os.path
import yaml

from pprint import pprint

# Ignore special node handling specified in the files
yaml.add_multi_constructor('tag:unity3d.com', lambda loader, suffix, node: loader.construct_mapping(node, deep=True))

_processTypes = ['.mat']


def BuildDatabase(opts):
    metadata = {}
    pathToGuid = {}
    
    for target in opts['dirs']:
        for root, dir, files in os.walk(target):
            for filename in files:
                base, ext = os.path.splitext(filename)
                metaPath = '%s/%s' % (root, filename)
                assetPath = '%s/%s' % (root, base)

                if ext == '.meta':
                    guid = None
                    
                    _, assetExt = os.path.splitext(base)
                    
                    with open(metaPath) as fptr:
                        data = yaml.load(fptr.read())
                        guid = data['guid']
                        pathToGuid[assetPath] = guid

                    #
                    if assetExt in _processTypes:
                        with open(assetPath) as fptr:
                            data = {}
                            for chunk in yaml.load_all(fptr.read()):
                                # print data
                                key = chunk.keys()[0]
                                data[key] = chunk[key]

                            #
                            data['path'] = assetPath
                            metadata[guid] = data

    #
    return metadata, pathToGuid
                    

def DictionaryToLuaTable(data):
    return data


def BuildMeshTextureMapping(db):
    # Run through all prefabs, looking for mesh references, and record mesh usage between meshes and textures
    mapping = {}

    for k,v in db.iteritems():
        mapping[k] = k

    return mapping


def Run(opts):
    db, pathToGuid = BuildDatabase(opts)
    pprint(pathToGuid)
    pprint(db)

    meshMapping = BuildMeshTextureMapping(db)
    pprint(DictionaryToLuaTable(meshMapping))

    
##################

dummyOpts = {
    'dirs': sys.argv[1:]
}

Run(dummyOpts)
    
