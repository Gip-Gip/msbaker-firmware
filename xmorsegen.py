file = open("text.txt", 'r')

text = file.read()

letters = []

for character in range(256):
    letters.append(chr(character))

commonality = [0] * 256

for character in text:
    commonality[letters.index(character)] += 1

rank = [0] * 256

for i in range(256):
    maxindex = commonality.index(max(commonality))
    rank[maxindex] = i
    commonality[maxindex] = -1;

#print(commonality)
print(rank)
