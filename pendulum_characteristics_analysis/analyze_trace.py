import pandas as pd
import matplotlib.pyplot as plt
import scipy.signal as signal
import numpy as np

filename = 'pendulum1_sail10_2.csv'

def calculate_damping_coefficient(df, peaks):
    """
    The amplitude of a damped pendulum is given by A(t) = A_0 * exp(-delta * t).
    This function calculates the damping coefficient delta from the first and last peak
    using the formula delta = - ln(A_1 / A_0) / t_1, where A_0 is the amplitude of the first peak,
    A_1 is the amplitude of the last peak and t_1 is the time of the last peak in relation to the 
    first peak.

    Source:
    https://www.ingenieurkurse.de/physik/schwingungen/gedaempfte-harmonische-schwingungen.html
    """

    A_0 = df['encoderValue'][peaks[0]]
    A_1 = df['encoderValue'][peaks[-1]]
    t_1 = df['time'][peaks[-1]] - df['time'][peaks[0]]
    delta = - np.log(A_1 / A_0) / t_1
    return delta
    

def read_and_prepare_data():
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
   
    # Remove all data before the first valid peak or valley
    firstPeakOrValley = min(peaks[0], valleys[0])
    df = df[firstPeakOrValley:]

    # Adjust time so that the first data point is at t=0
    df['time'] = df['time'] - df['time'].iloc[0]

    return df, peaks, valleys


def plot_data(df, peaks, valleys, delta):
    # Plot amplitude calculated from damping delta:
    A_0 = df['encoderValue'][abs(min(peaks[0], valleys[0]))]
    plt.plot(df['time'], A_0 * np.exp(-delta * df['time']), color='gray', label='Amplitude calculated from damping coefficient')
    plt.plot(df['time'], -A_0 * np.exp(-delta * df['time']), color='gray')

    # Plot the encoder values:
    plt.plot(df['time'], df['encoderValue'], label='Encoder value')

    # Plot the peaks and valleys:
    plt.scatter(df['time'][peaks], df['encoderValue'][peaks], c='r', label='Maxima')
    plt.scatter(df['time'][valleys], df['encoderValue'][valleys], c='g', label='Minima')

    plt.xlabel('time [seconds]')
    plt.ylabel('pendulum angle [degrees]')

    # Open plot in full screen:
    mng = plt.get_current_fig_manager()
    mng.window.state('zoomed')

    plt.legend()
    plt.show()


def main():
    df, peaks, valleys = read_and_prepare_data()

    print(f'Peaks: {df["encoderValue"][peaks].values}')
    print(f'Valleys: {df["encoderValue"][valleys].values}')

    # Calculate damping coefficient of pendulum
    delta = calculate_damping_coefficient(df, peaks)
    print(f'Damping coefficient: delta = {delta}')

    plot_data(df, peaks, valleys, delta)

if __name__ == '__main__':
    main()
