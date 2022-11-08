import shutil,shlex
from pathlib import Path

with open("rawflashcommand.txt","r") as ff:
    data = ff.read()
    spl = shlex.split(data)
    # spl = data.split(" ")
    # print(spl)
    outlist = ["python3 -m esptool"]
    lastwasport = False
    for item in spl[2:]:
        thepath = Path(item)
        if thepath.exists() and (thepath.suffix == ".bin"):  # if the item is a file, copy it and remove the path 
            shutil.copy2(thepath,"./")
            outlist.append(thepath.name)
        else:
            if lastwasport:
                outlist.append("<SERIALPORT>")
                lastwasport = False
            else:
                outlist.append(item)
                if item == "--port":
                    lastwasport = True            
    with open("flashcommand.sh","w") as ff2:
        ff2.write(" ".join(outlist))
        print(" ".join(outlist))

   