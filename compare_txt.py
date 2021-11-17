import filecmp

path1 = "readmymap.txt"
path2 = "../simulator/mapping.out"
f1 = open(path1, 'r')
f2 = open(path2, 'r')

i = 0

for line1 in f1:
    i += 1

    for line2 in f2:

        # matching line1 from both files
        if line1 == line2:
            # print IDENTICAL if similar
            print("Line ", i, ": IDENTICAL")
        else:
            print("Line ", i, ":")
            # else print that line from both files
            print("\tFile 1:", line1, end='')
            print("\tFile 2:", line2, end='')
        break

# closing files
f1.close()
f2.close()


# # Method 2
#
# with open(path1, 'r') as file1:
#     with open(path2, 'r') as file2:
#         same = set(file1).intersection(file2)
#
# same.discard('\n')
#
# with open('comp_result.txt', 'w') as file_out:
#     for line in same:
#         file_out.write(line)
#
# with open(path1, 'r') as file1:
#     with open(path2, 'r') as file2:
#         difference = set(file1).difference(file2)
#
# difference.discard('\n')
#
# with open('diff.txt', 'w') as file_out:
#     for line in difference:
#         file_out.write(line)