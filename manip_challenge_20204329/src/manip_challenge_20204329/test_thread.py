from threading import Thread
import time
from collections import deque

path_trajs = deque()
threads = []

def calc_path_trajs():
	global path_trajs
	for i in range(1000):
		path_trajs.append(i)
		print("{} is appended!".format(i))
		time.sleep(3)


def exec_path_trajs():
	global path_trajs
	while(threads[0].is_alive()):
		print("all path trajs: ", path_trajs)

		if(len(path_trajs)>0):
			print("target: ", path_trajs[0])
			path_trajs.popleft()
		time.sleep(1)
	print("exec_thread done")

if __name__ == '__main__':

	calc_thread = Thread(target = calc_path_trajs)
	exec_thread = Thread(target = exec_path_trajs)
	threads.append(calc_thread)
	threads.append(exec_thread)
	
	for t in threads:
		t.start()

	for t in threads:
		t.join()
	