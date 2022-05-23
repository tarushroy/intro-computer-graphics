import sys

if __name__ == "__main__":
    infilename = sys.argv[1]
    infile = open(infilename, "r")

    outfilename = infilename[:-4] + "_triangles.off"
    outfile = open(outfilename, "w")

    # off line
    infile.readline()

    # num line
    numline = infile.readline()
    numVertices, numFaces = int(numline.split()[0]), int(numline.split()[1])

    # read vertices
    vertices = [infile.readline() for i in range(numVertices)]

    # read faces
    faces = [infile.readline().split() for i in range(numFaces)]

    # convert faces
    newFaces = []
    for face in faces:
        if(face[0] == '3'):
            newFaces.append(face)
        elif(face[0] == '4'):
            # split into 2 triangles
            newFaces.append(['3', face[1], face[2], face[3]])
            newFaces.append(['3', face[1], face[3], face[4]])
    
    # write new off file
    outfile.write("OFF\n")
    outfile.write(str(numVertices) + " " + str(len(newFaces)) + " 0\n")
    outfile.writelines(vertices)
    for face in newFaces:
        outfile.write(" ".join(face) + "\n")
