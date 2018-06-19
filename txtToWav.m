audio1 = importaudiofromtxt('audio020.txt');
a = audio1(1:end-1);
T = audio1(end);
fs = floor(length(a) / T*1000);
f = 2*((a-min(a))/(max(a)-min(a))-0.5);

audiowrite('audio020.wav',f,fs);