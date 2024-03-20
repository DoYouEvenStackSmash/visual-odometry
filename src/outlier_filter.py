#!/usr/bin/python3
import numpy as np

def get_1d_gaussian(filter_size=3, sigma=1):
  x = np.linspace(-filter_size / 2, filter_size/2, filter_size)
  gaussian_filter_1d = np.exp(-x**2 / (2 * sigma ** 2)) / (np.sqrt(2 * np.pi) * sigma)
  return gaussian_filter_1d
  
def get_mean_filter(filter_size = 9):
  return np.ones((filter_size, 1)) / filter_size

def get_vec_filter(filtr):
  return filtr[:,np.newaxis]

def normalize(arx):
  #arx = array * (6618.75 - 1)
  #arx = (arx+1) / (7.5 * 882.5) * 95 + 1
  #arx = (7.5 * 882.5)/arx
  val = np.max(np.max(arx))
  oval = np.min(np.min(arx))
  arx = (arx - oval) / (val - oval) * -1 + 1
  arx = arx * (val - oval)
  return arx

def outlier_rejection(mean_deriv):
  # Calculate quartiles
  Q1 = np.percentile(mean_deriv[:,:], 25,axis=1)
  Q3 = np.percentile(mean_deriv[:,:],75, axis=1)

  # Calculate IQR
  IQR = Q3 - Q1
  threshold = 1.5 * IQR[:]
  lb = Q1 - threshold
  ub = Q3 + threshold
  # Define threshold (commonly, 1.5 * IQR)
  #for row in range(mean_deriv.shape[0]):
  #  mean_deriv[row,:] = lb[row] if mean_deriv[row,:] < lb[row] else mean_deriv[row,:]
  #  mean_deriv[row,:] = ub[row] if mean_deriv[row,:] > ub[row] else mean_deriv[row,:]
    #for col in range(mean_deriv.shape[1]):
      #mean_deriv[row,col] = lb[row] if mean_deriv[row,col] < lb[row] else mean_deriv[row,col]
      #mean_deriv[row,col] = ub[row] if mean_deriv[row,col] > ub[row] else mean_deriv[row,col]
  #mean_deriv[mean_deriv < lb[:]] = lb[:]
  #mean_deriv[mean_deriv > ub[:]] = ub[:]
  return np.clip(mean_deriv, lb[:,np.newaxis], ub[:,np.newaxis])

gauss_f = get_vec_filter(get_1d_gaussian(7))
gauss_delay = gauss_f.shape[0]
mean_f = get_vec_filter(get_mean_filter(9))
mean_delay = mean_f.shape[0]
deriv_f = get_vec_filter(np.array([-1,1]))
def find_obs(frame,lower_bound = 50, upper_bound = 350):
  conv = lambda x,h: np.apply_along_axis(lambda x: np.convolve(x, h.flatten(), mode='full'),axis=1,arr=x)
  frame = normalize(frame)
  
  # blur column
  # gaussian filter
  #gauss_f = get_vec_filter(get_1d_gaussian(7))
  blurred_frame = conv(frame, gauss_f)
  #gauss_delay = gauss_f.shape[0]
  # normalize
  #blurred_frame = (blurred_frame - np.min(np.min(blurred_frame))) / (np.max(np.max(blurred_frame)) - np.min(np.min(blurred_frame)))
  
  # apply window to frame
  bounded_frame = blurred_frame[:,lower_bound:upper_bound]
  
  # get mean filter
  #mean_f = get_vec_filter(get_mean_filter(9))
  #mean_delay = mean_f.shape[0]
  
  # apply mean filter
  mean_filtered_frame = conv(bounded_frame, mean_f)
  
  # get derivative filter
  #deriv_f = get_vec_filter(np.array([-1,1]))
  
  # get first derivative
  first_derivative = conv(mean_filtered_frame[:,:],deriv_f)#[:,1:-2]
  # return first_derivative

  # get mean delay
  # first_derivative = #first_derivative[:,]
  
  # outlier rejection 
  first_derivative = outlier_rejection((first_derivative))
  #print(first_derivative.shape)
  #first_derivative = normalize(first_derivative)
  obs_matrix = np.ones((frame.shape[0],upper_bound - lower_bound))
  #std_arr = np.std(first_derivative[:,:], axis=1)
  mean_arr = np.mean(first_derivative[:,:],axis=1)
  #print(obs_matrix.shape)
  for i in range(first_derivative.shape[0]):
    for j in range(mean_delay + gauss_delay,first_derivative.shape[1]-mean_delay - gauss_delay):
        if np.sign(first_derivative[i,j]) != np.sign(first_derivative[i,j-1]) or first_derivative[i,j] < mean_arr[i]/4:
          #obs_matrix[i,j:first_derivative.shape[1]-mean_delay - gauss_delay-1] = 0
          obs_matrix[i,j:-1] = 0
          break
          
  return obs_matrix#np.flip(obs_matrix,axis=1)

def get_free_space(rect_arr, arm):
  free_spaces = arm
  posns = np.argmax(rect_arr == 0, axis=1)
  posns[posns == 0] = rect_arr.shape[1]
  print(posns.shape)
  for i,p in enumerate(posns):
    if p != rect_arr.shape[1]:
      #free_spaces[i,50:50+p] = arm[i,50:50+p]
      free_spaces[i,50 + p:-1] = arm[i,50 + p -1]
  return free_spaces
 
  for i in range(arm.shape[0]):
    permj = 0
    free_spaces[i,0:49] = arm[i,0:49] 
    for j in range(50,arm.shape[1]):
      if j < 50:
        free_spaces[i,j] = arm[i,j]
      elif j > 349:
        free_spaces[i,j] = free_spaces[i,j-1]
      else:
        if rect_arr[i,j-50] != 0 and permj == 0:
          free_spaces[i,j] = arm[i,j]
        else:
          if permj == 0:
            permj = j - 1
          free_spaces[i,j:-1] = arm[i,permj]
          break
  return free_spaces

def get_markers(rect_arr):
  # markers = np.zeros((rect_arr.shape[1],1))
  markers = np.argmax(rect_arr[:,:] == 0,axis=1)
  return markers
  
        
      
  
  
  
  
  
  
  
  
  
  
  
  
