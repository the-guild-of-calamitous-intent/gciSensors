import pickle

def pickle_write(filename, data, info=None):
    if not isinstance(data, dict):
        print("ERROR: data is not a dictionary")
        return False
    if info is not None:
        if not isinstance(info, dict):
            print("ERROR: info is not a dictionary")
            return False
        data["info"] = info

    try:
        with open(filename, "wb") as fd:
            pickle.dump(data, fd)
    except:
        return False

    print(f"Wrote data to {filename}")

    return True


def pickle_read(filename):
    try:
        with open(filename, "rb") as fd:
            data = pickle.load(fd)
    except:
        return None

    print(f"Read {filename} with keys {data.keys()}")

    return data