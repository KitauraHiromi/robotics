import re
import numpy as np

pattern = ".*?(\d+, \d+, \d+, \d+, \d+, \d+).*"

x = []
with open("test.txt", "r") as f:
	for line in f:
		result = re.match(pattern, line)
		s = result.group(1)
		x.append(list(map(int, s.split(","))))

print(x)
print(np.array(x))
