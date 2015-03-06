
import numpy as np
import math 


def HSV_to_RGB(H,S,V):
	#http://www.rapidtables.com/convert/color/hsv-to-rgb.htm
	C = V * S
	X = C * (1 - np.abs(np.mod((H / 60.0),2.0) - 1))
	m = V - C
	if H>=0 and H<60:
		R=C
		G=X
		B=0
	elif H>=60 and H<120:
		R=X
		G=C
		B=0
	elif H>=120 and H<180:
		R=0
		G=C
		B=X
	elif H>=180 and H<240:
		R=0
		G=X
		B=C
	elif H>=240 and H<300:
		R=X
		G=0
		B=C
	elif H>=300 and H<=360:
		R=C
		G=0
		B=X
	return R+m,G+m,B+m

#-------------------------------------------------------------------------------------#
def norm_pdf_multivariate(x, mu, sigma):
     size = len(x)
     x_mu = np.matrix(x - mu)
     determ = np.linalg.det(sigma)
     if determ < 0.01:
         new_diag = np.diag(sigma).copy()
         new_diag[new_diag == 0] = 0.05
         new_sigma = np.matrix(np.diag(new_diag))
         determ = np.linalg.det(new_sigma)
         inv = new_sigma.I
     else:
         inv = sigma.I

     norm_const = 1.0/ (math.pow((2*np.pi),float(size)/2) * 
math.pow(determ,1.0/2))
     result = math.pow(math.e, -0.5 * (x_mu * inv * x_mu.T))
     return norm_const * result

#-------------------------------------------------------------------------------------#
def Update_dis_histogram_use_CK(hyp,o_qsr,sentence):

	# Use the consulidated knowledge
	
	flag = 0
	words = []
	o1 = 0
	o2 = 0
	for j in sentence.split(' '):
		if flag == 0:
			if j in hyp['objects_hyp']:
				o1 = hyp['objects_hyp'].index(j)
				flag = 1
		elif flag == 1:
			if j in hyp['objects_hyp']:
				o2 = hyp['objects_hyp'].index(j)
				flag = 0
				if o1 != o2:		#----------- check if o1 and o2 are different
					if o1>o2:
						kk=o2
						o2=o1
						o1=kk
					for k in words:
						if k not in hyp['valid_HSV_hyp']:
							hyp['hyp'][k]['counter_dis']+=1
							d = int(o_qsr[str(o1)+'-'+str(o2)+'-dis'])	#in m
							if d in hyp['hyp'][k]['hist_dis']:
								hyp['hyp'][k]['hist_dis'][d] += 1
								hyp['hyp'][k]['hist_dis_x'].append(d)
							else:
								hyp['hyp'][k]['hist_dis'][d] = 1
								hyp['hyp'][k]['hist_dis_x'].append(d)

				words = []
			else:
				words.append(j)
	return hyp

#-------------------------------------------------------------------------------------#
def Update_dis_histogram(hyp,o_qsr):

	for j in hyp['words']:
		if j not in hyp['valid_HSV_hyp']:
			hyp['hyp'][j]['counter_dis']+=1
			A = o_qsr['obj_number']

			for p in range(A-1):
				for k in range(p+1,A):
					d = int(o_qsr[str(o1)+'-'+str(o2)+'-dis'])	#in m
					
					if d in hyp['hyp'][j]['hist_dis']:
						hyp['hyp'][j]['hist_dis'][d] += 1
						hyp['hyp'][j]['hist_dis_x'].append(d)
					else:
						hyp['hyp'][j]['hist_dis'][d] = 1
						hyp['hyp'][j]['hist_dis_x'].append(d)	
	return hyp

#-------------------------------------------------------------------------------------#
def Compute_dis_hypotheses(hyp):
	hyp['valid_dis_hyp'] = []

	for j in hyp['hyp']:
	    if j not in hyp['valid_HSV_hyp'] and hyp['hyp'][j]['counter_dis']!=0:
		X = hyp['hyp'][j]['hist_dis_x']
		flag = 0

		for kk in [50,100]:
		    for jj in range(len(X)):
			x1 = X[jj]-kk
			x2 = X[jj]+kk

			hist_sum = 0
			keys = []
			for key in hyp['hyp'][j]['hist_dis']:
			    if key != 'x':
				if key>x1 and key<x2:
				    	hist_sum += hyp['hyp'][j]['hist_dis'][key]
					keys.append(key)

			# see if a cube in the hitogram can be valid to be a hypothesis
			if  hist_sum >= hyp['hyp'][j]['counter_dis']*.9:
				for key in keys:
					hyp['hyp'][j]['score_dis_x'].append(key)
				flag = 1

		    if flag == 1:	# hypothesis has been found
			hyp['valid_dis_hyp'].append(j)
			break

	for j in hyp['valid_dis_hyp']:

		hyp['hyp'][j]['dis_points_x'] = []
		X = hyp['hyp'][j]['hist_dis_x']
		X2 = hyp['hyp'][j]['score_dis_x']

		for p in range(len(X)):
			if X[p] in X2:
				d = X[p]/700.0
				hyp['hyp'][j]['dis_points_x'].append(d)


		hyp['hyp'][j]['dis_points_x_mean'] = np.mean(hyp['hyp'][j]['dis_points_x'])
		hyp['hyp'][j]['dis_points_x_std'] = np.std(hyp['hyp'][j]['dis_points_x'])
					
	return hyp
#-------------------------------------------------------------------------------------#
def Update_dir_histogram_use_CK(hyp,o_qsr,sentence):


	# Use the consulidated knowledge
	
	flag = 0
	words = []
	o1 = 0
	o2 = 0
	for j in sentence.split(' '):
		if flag == 0:
			if j in hyp['objects_hyp']:
				o1 = hyp['objects_hyp'].index(j)
				flag = 1
		elif flag == 1:
			if j in hyp['objects_hyp']:
				o2 = hyp['objects_hyp'].index(j)
				flag = 0
				if o1 != o2:		#----------- check if o1 and o2 are different
					if o1>o2:
						k=o2
						o2=o1
						o1=k
						o_qsr[str(o1)+'-'+str(o2)+'-ang'] = -o_qsr[str(o1)+'-'+str(o2)+'-ang'] 	# the other relation

					for k in words:
						if k not in hyp['valid_HSV_hyp']:
							hyp['hyp'][k]['counter_dir']+=1
							a = o_qsr[str(o1)+'-'+str(o2)+'-ang']*180/np.pi
							if a<0:
								a=a+360
							
							if a in hyp['hyp'][k]['hist_dir']:
								hyp['hyp'][k]['hist_dir'][a] += 1
								hyp['hyp'][k]['hist_dir_x'].append(a)
							else:
								hyp['hyp'][k]['hist_dir'][a] = 1
								hyp['hyp'][k]['hist_dir_x'].append(a)
				words = []
			else:
				words.append(j)
	return hyp
			

#-------------------------------------------------------------------------------------#
def Update_dir_histogram(hyp,o_qsr):


	for j in hyp['words']:
		if j not in hyp['valid_HSV_hyp']:
			hyp['hyp'][j]['counter_dir']+=1
			A = o_qsr['obj_number']

			for p in range(A-1):
				for k in range(p+1,A):
					a = o_qsr[str(0)+'-'+str(1)+'-ang']*180/np.pi
					if a<0:
						a=a+360
							
					if a in hyp['hyp'][j]['hist_dir']:
						hyp['hyp'][j]['hist_dir'][a] += 1
						hyp['hyp'][j]['hist_dir_x'].append(a)
					else:
						hyp['hyp'][j]['hist_dir'][a] = 1
						hyp['hyp'][j]['hist_dir_x'].append(a)	
	return hyp

#-------------------------------------------------------------------------------------
def Compute_dir_hypotheses(hyp):
	hyp['valid_dir_hyp'] = []
	window = 361

	for j in hyp['hyp']:
	    if j not in hyp['valid_HSV_hyp'] and hyp['hyp'][j]['counter_dir']!=0:

		X = hyp['hyp'][j]['hist_dir_x']
		flag = 0

		for kk in [45,90]:
		    for jj in range(len(X)):
			x1 = X[jj]-kk
			x2 = X[jj]+kk

			hist_sum = 0
			keys = []

			if x2 < (361):		# to make the circular thing continous
				for key in hyp['hyp'][j]['hist_dir']:
				    if key != 'x':
					if key>x1 and key<x2:
					    	hist_sum += hyp['hyp'][j]['hist_dir'][key]
						keys.append(key)
			else:
				for key in hyp['hyp'][j]['hist_dir']:
				    if key != 'x':
					if (key>x1 and key<361) or (key>0 and key<(x2-361)):
					    	hist_sum += hyp['hyp'][j]['hist_dir'][key]
						keys.append(key)

			# see if a cube in the hitogram can be valid to be a hypothesis
			if  hist_sum >= hyp['hyp'][j]['counter_dir']*.9:
				for key in keys:
					hyp['hyp'][j]['score_dir_x'].append(key)
				flag = 1

		    if flag == 1:	# hypothesis has been found
			hyp['valid_dir_hyp'].append(j)
			break


	for j in hyp['valid_dir_hyp']:

		hyp['hyp'][j]['dir_points_x'] = []
		X = hyp['hyp'][j]['hist_dir_x']
		X2 = hyp['hyp'][j]['score_dir_x']

		for p in range(len(X)):
			if X[p] in X2:
				d = X[p]
				hyp['hyp'][j]['dir_points_x'].append(d)

		hyp['hyp'][j]['dir_points_x_mean'] = circ_mean(hyp['hyp'][j]['dir_points_x'], low=0, high=360)
		hyp['hyp'][j]['dir_points_x_std'] = circ_std(hyp['hyp'][j]['dir_points_x'], low=0, high=360)
			
	return hyp
					
#-------------------------------------------------------------------------------------#
def Update_SPA_points(o_qsr,POINTS):

	A = o_qsr['obj_number']
	for p in range(A-1):
		for k in range(p+1,A):
			d = o_qsr[str(p)+'-'+str(k)+'-dis']/700
			a = o_qsr[str(p)+'-'+str(k)+'-ang']*180/np.pi
			xs,ys,zs = HSV_to_XYZ(a, d, 0.5)		# in this case Z is always to zero
			x = int(xs[0]*100+100)
			y = int(ys[0]*100+100)
			z = int(zs[0]*100)
			POINTS.append([xs,ys,zs,1.0,1.0,1.0])
				
	return POINTS


#-------------------------------------------------------------------------------------#
def Test_HSV_Hypotheses(hyp,o_color):
	hyp['objects'] = {}
	hyp['objects_hyp'] = []
	for i in range(len( o_color['H'])):
		H = o_color['H'][i]
		S = o_color['S'][i]
		V = o_color['V'][i]
		x,y,z = HSV_to_XYZ(H, S, V)
		A = np.matrix([x[0],y[0],z[0]])
		hyp['objects'][i] = ''
		var = 0
		for j in hyp['valid_HSV_hyp']:

			X = hyp['hyp'][j]['HSV_points_x_mean']
			Y = hyp['hyp'][j]['HSV_points_y_mean']
			Z = hyp['hyp'][j]['HSV_points_z_mean']

			Xs= hyp['hyp'][j]['HSV_points_x_std']
			Ys= hyp['hyp'][j]['HSV_points_y_std']
			Zs= hyp['hyp'][j]['HSV_points_z_std']

			B = np.matrix([X,Y,Z])
			result = norm_pdf_multivariate(A, B, np.matrix([[Xs, 0, 0], [0, Ys, 0], [0, 0, Zs]]))

			if var<result:
				hyp['objects'][i] = j
				var = result

		hyp['objects_hyp'].append(hyp['objects'][i])

	return hyp


#-------------------------------------------------------------------------------------#
def Test_Relation_Hypotheses(hyp,o_qsr):
	hyp['relations'] = {}
	hyp['relations']['dis'] = {}
	hyp['relations']['dir'] = {}
	hyp['relations_hyp'] = {}
	hyp['relations_hyp']['dis'] = []
	hyp['relations_hyp']['dir'] = []

	if 'valid_dis_hyp' in hyp:
	    if hyp['valid_dis_hyp'] != []:
		for i in range(len( o_qsr[0])):
			hyp['relations']['dis'][i] = ''
			A = np.matrix([o_qsr[0][i]])
		
			var = 0
			for j in hyp['valid_dis_hyp']:
				X = hyp['hyp'][j]['dis_points_x_mean']
				Xs= hyp['hyp'][j]['dis_points_x_std']
				B = np.matrix([X])
				result = norm_pdf_multivariate(A, B, np.matrix([[Xs]]))
				if var<result:
					hyp['relations']['dis'][i] = j
					var = result

			hyp['relations_hyp']['dis'].append(hyp['relations']['dis'][i])
	    else:
		# initilize with empty
		for i in range(len( o_qsr[0])):
			hyp['relations_hyp']['dis'].append('')


	    if hyp['valid_dir_hyp'] != []:
		for i in range(len( o_qsr[1])):
			hyp['relations']['dir'][i] = ''
			A = np.matrix([o_qsr[1][i]*180/np.pi])
		
			var = 0
			for j in hyp['valid_dir_hyp']:
				X = hyp['hyp'][j]['dir_points_x_mean']
				Xs= hyp['hyp'][j]['dir_points_x_std']
				B = np.matrix([X])
				result = norm_pdf_multivariate(A, B, np.matrix([[Xs]]))
				if var<result:
					hyp['relations']['dir'][i] = j
					var = result
				########## ASK ERIS TO GIVE CIRCULER NORM ########
				if j=='right':
					X = hyp['hyp'][j]['dir_points_x_mean']-360
					Xs= hyp['hyp'][j]['dir_points_x_std']
					B = np.matrix([X])
					result = norm_pdf_multivariate(A, B, np.matrix([[Xs]]))
					if var<result:
						hyp['relations']['dir'][i] = j
						var = result
					

			hyp['relations_hyp']['dir'].append(hyp['relations']['dir'][i])
	    else:
		# initilize with empty
		for i in range(len( o_qsr[1])):
			hyp['relations_hyp']['dir'].append('')

	else:
		# initilize with empty
		for i in range(len( o_qsr[0])):
			hyp['relations_hyp']['dis'].append('')
			hyp['relations_hyp']['dir'].append('')
	return hyp

#-------------------------------------------------------------------------------------#
def Update_HSV_histogram(hyp,o_color,POINTS):

	for j in hyp['words']:
		hyp['hyp'][j]['score_HSV_x'] = []
		hyp['hyp'][j]['score_HSV_y'] = []
		hyp['hyp'][j]['score_HSV_z'] = []
		for p in range(len(o_color['H'])):
			H = o_color['H'][p]
			S = o_color['S'][p]
			V = o_color['V'][p]
			xs,ys,zs = HSV_to_XYZ(H, S, V)
			x = int(xs[0]*100+100)
			y = int(ys[0]*100+100)
			z = int(zs[0]*100)
			if (x,y,z) in hyp['hyp'][j]['hist_HSV']:
				hyp['hyp'][j]['hist_HSV'][(x,y,z)] += 1
			else:
				hyp['hyp'][j]['hist_HSV'][(x,y,z)] = 1
				hyp['hyp'][j]['hist_HSV_x'].append(x)
				hyp['hyp'][j]['hist_HSV_y'].append(y)
				hyp['hyp'][j]['hist_HSV_z'].append(z)

			R,G,B = HSV_to_RGB(H,S,V)
			POINTS.append([xs,ys,zs,R,G,B])
	return hyp,POINTS

#-------------------------------------------------------------------------------------#
def Compute_HSV_hypotheses(hyp):

	#============== create a set of believes for HSV ===============#
	hyp['valid_HSV_hyp'] = []

	for j in hyp['hyp']:
		X = hyp['hyp'][j]['hist_HSV_x']
		Y = hyp['hyp'][j]['hist_HSV_y']
		Z = hyp['hyp'][j]['hist_HSV_z']
		flag = 0
		for kk in [15,20,30]:
		    for jj in range(len(X)):
			x1 = X[jj]-kk
			x2 = X[jj]+kk
			y1 = Y[jj]-kk
			y2 = Y[jj]+kk
			z1 = Z[jj]-kk
			z2 = Z[jj]+kk

			hist_sum = 0
			keys = []
			for key in hyp['hyp'][j]['hist_HSV']:
			    if key[0] != 'x':
				if key[0]>x1 and key[0]<x2 and key[1]>y1 and key[1]<y2 and key[2]>z1 and key[2]<z2:
				    	hist_sum += hyp['hyp'][j]['hist_HSV'][key]
					keys.append(key)

			# see if a cube in the hitogram can be valid to be a hypothesis
			if  hist_sum >= hyp['hyp'][j]['counter_HSV']*.9:
				for key in keys:
					hyp['hyp'][j]['score_HSV_x'].append(key[0])
					hyp['hyp'][j]['score_HSV_y'].append(key[1])
					hyp['hyp'][j]['score_HSV_z'].append(key[2])
				flag = 1

		    if flag == 1:	# hypothesis has been found
			hyp['valid_HSV_hyp'].append(j)
			break

	for j in hyp['valid_HSV_hyp']:
		hyp['hyp'][j]['HSV_points_x'] = []
		hyp['hyp'][j]['HSV_points_y'] = []
		hyp['hyp'][j]['HSV_points_z'] = []

		X = hyp['hyp'][j]['hist_HSV_x']
		Y = hyp['hyp'][j]['hist_HSV_y']
		Z = hyp['hyp'][j]['hist_HSV_z']

		X2 = hyp['hyp'][j]['score_HSV_x']
		Y2 = hyp['hyp'][j]['score_HSV_y']
		Z2 = hyp['hyp'][j]['score_HSV_z']

		for p in range(len(X)):
			if X[p] in X2 and Y[p] in Y2 and Z[p] in Z2:
				hyp['hyp'][j]['HSV_points_x'].append((X[p]-100)/100.0)
				hyp['hyp'][j]['HSV_points_y'].append((Y[p]-100)/100.0)
				hyp['hyp'][j]['HSV_points_z'].append(Z[p]/100.0)


		hyp['hyp'][j]['HSV_points_x_mean'] = np.mean(hyp['hyp'][j]['HSV_points_x'])
		hyp['hyp'][j]['HSV_points_y_mean'] = np.mean(hyp['hyp'][j]['HSV_points_y'])
		hyp['hyp'][j]['HSV_points_z_mean'] = np.mean(hyp['hyp'][j]['HSV_points_z'])

		hyp['hyp'][j]['HSV_points_x_std'] = np.std(hyp['hyp'][j]['HSV_points_x'])
		hyp['hyp'][j]['HSV_points_y_std'] = np.std(hyp['hyp'][j]['HSV_points_y'])
		hyp['hyp'][j]['HSV_points_z_std'] = np.std(hyp['hyp'][j]['HSV_points_z'])
		
	return hyp

#-------------------------------------------------------------------------------------#
def sentence_parsing(hyp,sentence):
	#------------- create a list of all words ---------------#
	hyp['words'] = []
	for j in sentence.split(' '):
		if j not in hyp['hyp']:
			hyp['hyp'][j] = {}
			hyp['hyp'][j]['counter_HSV'] = 0	
			hyp['hyp'][j]['hist_HSV'] = {}
			hyp['hyp'][j]['hist_HSV_x'] = []
			hyp['hyp'][j]['hist_HSV_y'] = []
			hyp['hyp'][j]['hist_HSV_z'] = []
			hyp['hyp'][j]['score_HSV'] = {}
			hyp['hyp'][j]['score_HSV_x'] = []
			hyp['hyp'][j]['score_HSV_y'] = []
			hyp['hyp'][j]['score_HSV_z'] = []

			hyp['hyp'][j]['counter_dis'] = 0	
			#hyp['hyp'][j]['hist_dis'] = np.zeros(shape=(1000),dtype=np.int16)		
			#hyp['hyp'][j]['score_dis'] = np.zeros(shape=(1000),dtype=np.int16)
			hyp['hyp'][j]['hist_dis'] = {}
			hyp['hyp'][j]['hist_dis_x'] = []
			hyp['hyp'][j]['score_dis'] = {}
			hyp['hyp'][j]['score_dis_x'] = []		

			hyp['hyp'][j]['counter_dir'] = 0
			#hyp['hyp'][j]['hist_dir'] = np.zeros(shape=(360),dtype=np.int16)
			#hyp['hyp'][j]['score_dir'] = np.zeros(shape=(360),dtype=np.int16)
			hyp['hyp'][j]['hist_dir'] = {}
			hyp['hyp'][j]['hist_dir_x'] = []
			hyp['hyp'][j]['score_dir'] = {}
			hyp['hyp'][j]['score_dir_x'] = []		

		if j not in hyp['words']:
			hyp['words'].append(j)	

	# -- add 1 to counter HSV for as many times repeated --#
	for j in hyp['words']:
		hyp['hyp'][j]['counter_HSV']+=1

	return hyp



#-------------------------------------------------------------------------------------#
def HSV_to_XYZ(H, S, V):
	x = [S*np.cos(H*np.pi/180.0)]
	y = [S*np.sin(H*np.pi/180.0)]
	z = [V]
	return x,y,z

#-------------------------------------------------------------------------------------#
def check(b1,b2,a,b):
	if b1<a:
		b1=a
	return b1,b2

#-------------------------------------------------------------------------------------#
def circ_mean(samples, low=0, high=2*np.pi):
     """Compute the circular mean for samples assumed to be in the range 
[low to high]
     """
     ang = (np.array(samples) - low)*2*np.pi / (high-low)
     mu = np.angle(np.mean(np.exp(1j*ang)))
     if (mu < 0):
         mu = mu + 2*np.pi
     return mu * (high-low)/(2.0*np.pi) + low

#-------------------------------------------------------------------------------------#
def circ_std(samples, low=0, high=2*np.pi):
     """Compute the circular standard deviation for samples assumed to 
be in the range [low to high]
     """
     ang = (np.array(samples) - low)*2*np.pi / (high-low)
     R = np.mean(np.exp(1j*ang))
     V = 1-abs(R)
     return np.sqrt(V) * (high-low)/(2.0*np.pi)
