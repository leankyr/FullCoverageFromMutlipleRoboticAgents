#!/usr/bin/env python

from __future__ import print_function

import datetime
import time

# function that makes the transform
def date_to_stamp(date):

    print ('date original: ', date)
    # split the original string
    date_split = date.split()
    # turn it into a list
    date = list(date_split)
    # Change the months in Words into Numbers
    # Works only for April and May
    for word in range(len(date)):
        if date[word] ==  'May':
            date[word] = '5'
        if date[word] ==  'Apr':
            date[word] = '4'

    # Remove the days from the date eg. Mon, Wed, Sat
    for word in range(len(date)):
        if str.isalpha(date[word]):
            date.remove(date[word])
            break

    # keep the coverage value 
    cov = date[0]

    # remove it from the date
    date.remove(date[0])

    # split the hourtime
    split_time = date[2].split(':')

    date[2] = split_time
    
    # extract the date object
    dt = datetime.datetime(int(date[3]), int(date[0]), int(date[1]), 
                    int(date[2][0]), int(date[2][1]), int(date[2][2]))

    print ('date object is:', dt)

    # Convert it into timestamp
    timestamp = int(time.mktime(dt.timetuple()))
    
    # Return the coverage value and the unix timestamp
    # return list
    ret = list()
    ret.append(float(cov))
    ret.append(timestamp)
    return ret 

if __name__ == '__main__':
    
    # date = '0.012 Mon Apr  28 22:23:15 2019'
    with open("exp1_med_map.txt") as file_obj:
        dates = file_obj.readlines()

    # input vector for matlab
    mat_vec = list()
    for i in range(len(dates)):    
        mat_vec.append(date_to_stamp(dates[i]))
        
    # change slide the first count to zero   
    transform = mat_vec[0][1]
    for i in range(len(mat_vec)):
        mat_vec[i][1] = mat_vec[i][1] - transform

    # print the output and hope that matlab will like it 
    for i in range(len(mat_vec)):
        print ('mat_vec for ', i + 1, ' is ', mat_vec[i])






