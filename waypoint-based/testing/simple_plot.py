import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

filenames = ["zz_noiseless_2.csv","zz_noisy_2.csv","zz_noiseless_10.csv","zz_noisy_10.csv","circle_noiseless_1.csv","circle_noisy_1.csv","circle_noiseless_5.csv","circle_noisy_5.csv"]

def get_xyz(filename):
    data = {'tracker': {'x':[],'y':[],'z':[]},'vehicle':{'x':[],'y':[],'z':[]}}
    file_csv = open("csv/"+filename)
    data_csv = csv.reader(file_csv)
    list_csv =  list(data_csv)[1:]
    home = [0.0]*6
    for i in list_csv:
        data['tracker']['x'].append(float(i[0]) - float(home[0]))
        data['vehicle']['x'].append(float(i[1]) - float(home[1]))
        data['tracker']['y'].append(float(i[2]) - float(home[2]))
        data['vehicle']['y'].append(float(i[3]) - float(home[3]))
        data['tracker']['z'].append(float(i[4]) - float(home[4]))
        data['vehicle']['z'].append(float(i[5]) - float(home[5]))

    return data

def plot_data(filename):
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    new_filename = "images/"+filename[:-4]+'_plot.png'
    data = get_xyz(filename)
    ax.plot(data['tracker']['x'],data['tracker']['y'],data['tracker']['z'])
    ax.plot(data['vehicle']['x'],data['vehicle']['y'],data['vehicle']['z'])
    plt.savefig(new_filename)

if __name__ == '__main__':
    for x in filenames:
        plot_data(x)
    