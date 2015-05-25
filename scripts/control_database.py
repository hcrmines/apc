#!/usr/bin/env python

import json
import os
import copy

def prioritizeWorkOrder():
	json_data = open('apc.json')
	data = json.load(json_data)
	bin_contents = data['bin_contents']
	work_order = data['work_order']

	#CONST_BIN_NAMES = ['bin_A',
	#                   'bin_B',
	#                   'bin_C',
	#                   'bin_D',
	#                   'bin_E',
	#                   'bin_F',
	#                   'bin_G',
	#                   'bin_H',
	#                   'bin_I',
	#                   'bin_J',
	#                   'bin_K',
	#                   'bin_L']

	# dictionary with item name as key and [num grips, bonus] as value
	CONST_ITEM_details = {"oreo_mega_stuf":[0, 0],
	                    "champion_copper_plus_spark_plug":[3, 0],
	                    "expo_dry_erase_board_eraser":[0, 0],
	                    "kong_duck_dog_toy":[3, 1],
	                    "genuine_joe_plastic_stir_sticks":[0, 0],
	                    "munchkin_white_hot_duck_bath_toy":[2.5, 1],
	                    "crayola_64_ct":[1, 0],
	                    "mommys_helper_outlet_plugs":[0, 0],
	                    "sharpie_accent_tank_style_highlighters":[1, 0],
	                    "kong_air_dog_squeakair_tennis_ball":[0, 1],
	                    "stanley_66_052":[0.5, 3],
	                    "safety_works_safety_glasses":[2.5, 1],
	                    "dr_browns_bottle_brush":[2, 2],
	                    "laugh_out_loud_joke_book":[0.2, 3],
	                    "cheezit_big_original":[0, 0],
	                    "paper_mate_12_count_mirado_black_warrior":[2, 0],
	                    "feline_greenies_dental_treats":[1, 0],
	                    "elmers_washable_no_run_school_glue":[3, 0],
	                    "mead_index_cards":[0.5, 0],
	                    "rolodex_jumbo_pencil_cup":[0, 0],
	                    "first_years_take_and_toss_straw_cup":[1, 0],
	                    "highland_6539_self_stick_notes":[2, 0],
	                    "mark_twain_huckleberry_finn":[0.5, 3],
	                    "kyjen_squeakin_eggs_plush_puppies":[3, 1],
	                    "kong_sitting_frog_dog_toy":[3, 1]}

	# we have no gripping test data for oreo_mega_stuf, expo_dry_erase_board_eraser, and
	#kong_air_dog_squeakair_tennis_ball

	# the grips are for small gripper. mommys_helper_outlet_plugs, rolodex_jumbo_pencil_cup,
	# cheezit_big_original, and first_years_take_and_toss_straw_cup all have a gripping score
	#of 2 using large gripper

	single_items = {}
	double_items = {}
	multi_items = {}
	for bin_name in bin_contents:
		if len(bin_contents[bin_name]) == 1:
			single_items[bin_name] = bin_contents[bin_name]
		if len(bin_contents[bin_name]) == 2:
			double_items[bin_name] = bin_contents[bin_name]
		if len(bin_contents[bin_name]) > 2:
			multi_items[bin_name] = bin_contents[bin_name]

	scored_work_order = []
	for order in work_order:
		grasp = CONST_ITEM_details[order['item']][0]
		bonus = CONST_ITEM_details[order['item']][1]
		if order['item'] in [item for sublist in single_items.values() for item in sublist]:
			bin_type = 3
		elif order['item'] in [item for sublist in double_items.values() for item in sublist]:
			bin_type = 1
		else:
			bin_type = 0
		score = ((0.5*bonus+bin_type)*grasp/13.5)
		scored_work_order.append([order['item'], order['bin'][-1], score, 0]) #Currently uppercase
	scored_work_order_decreasing = sorted(scored_work_order, key=lambda x: x[2], reverse=True)
	#pprint(scored_work_order_decreasing)
	json_data.close()
	return scored_work_order_decreasing

class StateKeeper:

	fileName = 'savefile.txt'

	states = []
	#data
	targets = []
	#tuple('objname', bin, grabScore, absoluteScore)
	completed = []
	#tuple('objname', bin)

	def __init__(self):
		if os.path.exists(self.fileName):
			f =  open(self.fileName, 'r')
			x = json.load(f)
			if x:
				self.completed = x
		self.targets = prioritizeWorkOrder()


	def addState(self, data):
		tempLambda = lambda d: d.obj_id==data.obj_id and \
			d.job_number!=data.job_number #and /
			#d.bin_loc==data.bin_loc

		objectsFound = filter(tempLambda, self.states)
		for eachObject in objectsFound:
			self.states.remove(eachObject)

		self.states.append(copy.deepcopy(data))

	def successState(self, data):
		self.completed.append((data.obj_id, data.bin_loc))
		f =  open(self.fileName, 'w')
		json.dump(self.completed, f)

	def updateConfidence(self, data):
		tempLambda = lambda d: d.obj_id==data.obj_id and \
			d.job_number==data.job_number #and \
			#d.bin_loc==data.bin_loc

		objectsFound = filter(tempLambda, self.states)
		for eachObject in objectsFound:
			eachObject.confidence = 0


	def resetScores():
		self.targets = prioritizeWorkOrder()


	def getNextTarget(self):
		maxScore = -1
		maxObject = 0
		maxTarget = 0
		#print self.targets
		for target in self.targets:
			for state in self.states:
				if state.obj_id in objects.keys() and objects[state.obj_id]==target[0]:
				#and state.bin_loc==target[1]: #objects match #TODO
					bonus = 0
					if not state.is3d: #if 2D
						bonus = 100
					target[3] = (target[2]*(state.confidence+1))+bonus-(target[3]*.5)
							#abs = grabScore+confidence + bonus - .5xattempt count
					if target[3]>maxScore:
						maxScore=target[3]
						maxObject=state
			#end for
			target[3] = (target[2]*(1))
			if target[3]>maxScore:
				maxScore=target[3]
				maxTarget = target
				maxObject=target[1]

		maxTarget[3] += 1
		return maxObject, maxTarget


objects = {1:"champion_copper_plus_spark_plug",
    2:"cheezit_big_original",
    3:"crayola_64_ct",
    4:"dove_beauty_bar ",
    5:"dr_browns_bottle_brush",
    6:"elmers_washable_no_run_school_glue",
    7:"expo_dry_erase_board_eraser",
    8:"feline_greenies_dental_treats",
    9:"first_years_take_and_toss_straw_cup",
    10:"genuine_joe_plastic_stir_sticks",
    11:"highland_6539_self_stick_notes",
    12:"kong_air_dog_squeakair_tennis_ball",
    13:"kong_duck_dog_toy",
    14:"kong_sitting_frog_dog_toy",
    15:"kyjen_squeakin_eggs_plush_puppies",
    16:"laugh_out_loud_joke_book",
    17:"mark_twain_huckleberry_finn",
    18:"mead_index_cards",
    19:"mommys_helper_outlet_plugs",
    20:"munchkin_white_hot_duck_bath_toy",
    21:"one_with_nature_soap_dead_sea_mud ",
    22:"oreo_mega_stuf",
    23:"paper_mate_12_count_mirado_black_warrior",
    24:"rolodex_jumbo_pencil_cup",
    25:"safety_works_safety_glasses",
    26:"sharpie_accent_tank_style_highlighters",
    27:"stanley_66_052"
}
