import pandas as pd
import matplotlib.pyplot as plt
import scipy.signal as signal

filename = 'log2.csv'

# read semicolon-separated values from log1.csv and store them in a Pandas dataframe
df = pd.read_csv(filename, sep=';')

# convert time column to seconds
df['time'] = df['timeMicros'] / 1000000

# convert encoderValue column to degrees
df['encoderValue'] = (df['encoderValue'] / 2400) * 360

# Find extrema
peaks = signal.find_peaks(df['encoderValue'])[0]
valleys = signal.find_peaks(-df['encoderValue'])[0]

# Remove the first peak and valley as the hand still touches the pendulum at that time
peaks = peaks[1:]
valleys = valleys[1:]

print(f'Peaks: {df["encoderValue"][peaks].values}')
print(f'Valleys: {df["encoderValue"][valleys].values}')

# Remove all data before the first valid peak or valley
firstPeakOrValley = min(peaks[0], valleys[0])
df = df[firstPeakOrValley:]

# Adjust time so that the first data point is at t=0
df['time'] = df['time'] - df['time'].iloc[0]

# create a plot of the data and the extrema
plt.plot(df['time'], df['encoderValue'])
plt.scatter(df['time'][peaks], df['encoderValue'][peaks], c='r')
plt.scatter(df['time'][valleys], df['encoderValue'][valleys], c='g')
plt.xlabel('time [seconds]')
plt.ylabel('pendulum angle [degrees]')
plt.show()
