function metrics = postprocess_s28(dataDir,outDir)
%POSTPROCESS_S28 Compute and plot all four ADC tests in MATLAB from VCD samples.
% dataDir contains 01_resolution.csv, 02_range.csv, 03_static.csv,
% 04_dynamic.csv and test_cases.json. Each CSV is an extraction of the
% archived VCD at CLKOUT1 rising edges; vin_diff_v is captured at CLK_IP fall.
% No simulated, interpolated or ideal output code is inserted into the data.
if nargin<1
    root=fileparts(fileparts(mfilename('fullpath')));
    dataDir=fullfile(root,'doc','matlab','data');
end
if nargin<2, outDir=fullfile(fileparts(dataDir),'results'); end
if ~isfolder(outDir), mkdir(outDir); end
diary(fullfile(outDir,'matlab_execution.txt'));
guard=onCleanup(@()diary('off')); %#ok<NASGU>
fprintf('MATLAB %s\n',version);
fprintf('Postprocessing started: %s\n',char(datetime('now')));
cfg=jsondecode(fileread(fullfile(dataDir,'test_cases.json')));
fs=cfg.sample_rate_hz; M=cfg.M; k=cfg.k; discard=cfg.startup_discard_samples;
vref=cfg.cdac_reference_v; lsb=2*vref/64;
assert(gcd(k,M)==1 && mod(M,2)==0,'Coherent sampling configuration required.');
metrics=struct('engine','MATLAB','version',version,'sample_rate_hz',fs,...
    'M',M,'k',k,'input_frequency_hz',k*fs/M,'discard',discard,...
    'harmonic_order',cfg.harmonic_order,'bandwidth_hz',cfg.analysis_bandwidth_hz,...
    'window','rectangular','normalization','one-sided abs(fft(x)/M)^2; interior bins doubled');
blue=[0 .32 .65]; orange=[.82 .25 .04];

%% 1. Resolution: measured first/last internal thresholds and uncertainty.
R=readSamples(fullfile(dataDir,'01_resolution.csv'),fs);
[m1,g1]=dcMetrics(R,lsb); metrics.resolution=m1;
writetable(g1,fullfile(outDir,'01_resolution_staircase.csv'));
fig=makeFigure; tl=tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
ax=nexttile(tl); plot(ax,R.sample_time_s*1e6,R.vin_diff_v,'Color',blue,'LineWidth',1);
style(ax,'Time (\mus)','Differential input (V)','ADC resolution: recorded input');
ax=nexttile(tl); stairs(ax,R.sample_time_s*1e6,R.code,'Color',orange,'LineWidth',1);
style(ax,'Time (\mus)','Output code',sprintf('64 codes (0-63); average LSB = %.6f mV',m1.average_lsb_v*1000));
ylim(ax,[-2 65]); yticks(ax,[0 16 32 48 63]);
savePlot(fig,outDir,'01_resolution_resolution_ramp');

%% 2. Range: independent full ramp, boundary brackets and saturation.
R=readSamples(fullfile(dataDir,'02_range.csv'),fs);
[m2,g2]=dcMetrics(R,lsb); metrics.range=m2;
writetable(g2,fullfile(outDir,'02_range_staircase.csv'));
firstMax=find(g2.vin_diff_v>=1.1-1e-12,1); ramp=g2(1:firstMax,:);
fig=makeFigure; tl=tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
ax=nexttile(tl); stairs(ax,ramp.vin_diff_v,ramp.code,'Color',blue,'LineWidth',1.1);
style(ax,'Differential input (V)','Output code','ADC input range and saturation');
xlim(ax,[-1.1 1.1]); ylim(ax,[-2 65]); yticks(ax,[0 16 32 48 63]);
xline(ax,m2.v1_v,'--','V_1'); xline(ax,m2.v63_v,'--','V_{63}');
ax=nexttile(tl); counts=histcounts(ramp.code,-.5:1:63.5);
bar(ax,0:63,counts,1,'FaceColor',blue,'EdgeColor','none');
style(ax,'Output code','Number of ramp levels','Full-ramp code coverage'); xlim(ax,[-1 64]);
savePlot(fig,outDir,'02_range_range_scan');

%% 3. Independent code-density analysis of the retained conversion records.
S=readSamples(fullfile(dataDir,'03_static.csv'),fs);
assert(height(S)>=discard+M,'Incomplete static record'); S=S(discard+(1:M),:);
n=(0:M-1)'; X=[ones(M,1),sin(2*pi*k*n/M),cos(2*pi*k*n/M)];
q=X\S.vin_diff_v; dc=q(1); amp=hypot(q(2),q(3));
counts=histcounts(S.code,-.5:1:63.5)'; cum=cumsum(counts);
transitionCode=(1:63)'; validTransition=cum(1:63)>0 & cum(1:63)<M;
transition=nan(63,1);
transition(validTransition)=dc-amp*cos(pi*cum(find(validTransition))/M);
validIds=find(validTransition); first=validIds(1); last=validIds(end);
endpointLSB=(transition(last)-transition(first))/(last-first);
endpointLine=transition(first)+(transitionCode-first)*endpointLSB;
inl=(transition-endpointLine)/lsb;
codes=(0:63)'; ideal=nan(64,1); dnl=nan(64,1);
% Only codes with two observed bounding transitions support a full bin width.
validCodes=(first:last-1)';
lower=transition(first)+(validCodes-first)*endpointLSB;
upper=lower+endpointLSB;
cdf=@(v) .5+asin(max(-1,min(1,(v-dc)/amp)))/pi;
ideal(validCodes+1)=M*(cdf(upper)-cdf(lower));
dnl(validCodes+1)=counts(validCodes+1)./ideal(validCodes+1)-1;
metrics.static=struct('fitted_peak_v',amp,'fitted_dc_v',dc,...
    'valid_code_min',validCodes(1),'valid_code_max',validCodes(end),...
    'first_measured_transition',first,'last_measured_transition',last,...
    'dnl_min_lsb',min(dnl,[],'omitnan'),'dnl_max_lsb',max(dnl,[],'omitnan'),...
    'inl_min_lsb',min(inl,[],'omitnan'),'inl_max_lsb',max(inl,[],'omitnan'),...
    'missing_valid_codes',sum(counts(validCodes+1)==0));
writetable(table(codes,counts,ideal,dnl),fullfile(outDir,'03_static_dnl.csv'));
writetable(table(transitionCode,transition,endpointLine,inl,validTransition),fullfile(outDir,'03_static_inl.csv'));
fig=makeFigure; tl=tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
ax=nexttile(tl); plot(ax,validCodes,dnl(validCodes+1),'o-','Color',blue,'MarkerSize',3,'LineWidth',1);
style(ax,'Output code','DNL (LSB)','ADC static linearity: sine code density');
ylim(ax,[-.6 .6]); xlim(ax,[0 63]); yline(ax,.5,'k--');yline(ax,-.5,'k--');yline(ax,0,'k:');
ax=nexttile(tl); plot(ax,transitionCode,inl,'o-','Color',orange,'MarkerSize',3,'LineWidth',1);
style(ax,'Transition index','INL (LSB)','Endpoint INL using measured transitions');
ylim(ax,[-.6 .6]); xlim(ax,[0 63]); yline(ax,.5,'k--');yline(ax,-.5,'k--');yline(ax,0,'k:');
savePlot(fig,outDir,'03_static_dnl_inl');

%% 4. FFT computed directly from actual output codes; fold each harmonic.
D=readSamples(fullfile(dataDir,'04_dynamic.csv'),fs);
assert(height(D)>=discard+M,'Incomplete dynamic record'); D=D(discard+(1:M),:);
x=double(D.code); x=x-mean(x); Y=fft(x)/M;
P=abs(Y(1:M/2+1)).^2; P(2:end-1)=2*P(2:end-1);
f=(0:M/2)'*fs/M; fund=P(k+1); orders=(2:cfg.harmonic_order)';
rawBins=mod(orders*k,M); bins=min(rawBins,M-rawBins);
assert(numel(unique(bins))==numel(bins)&&all(bins~=0 & bins~=k),'Aliased harmonic bins overlap');
band=f>0 & f<=cfg.analysis_bandwidth_hz;
harmonicPower=P(bins+1); harmonicPower(f(bins+1)>cfg.analysis_bandwidth_hz)=0;
noiseMask=band; noiseMask(k+1)=false; noiseMask(bins+1)=false;
noise=sum(P(noiseMask)); thd=10*log10(sum(harmonicPower)/fund);
snr=10*log10(fund/noise); sndr=10*log10(fund/(noise+sum(harmonicPower)));
enob=(sndr-1.76)/6.02;
metrics.dynamic=struct('snr_db',snr,'thd_db',thd,'sndr_db',sndr,'enob_bit',enob);
relativeDB=10*log10(max(P/fund,realmin));
writetable(table(f,P,relativeDB),fullfile(outDir,'04_dynamic_spectrum.csv'));
writetable(table(orders,bins,f(bins+1),harmonicPower,'VariableNames',...
    {'harmonic_order','fft_bin','frequency_hz','power_code_squared'}),fullfile(outDir,'04_dynamic_harmonics.csv'));
fig=makeFigure; tl=tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
ax=nexttile(tl); plot(ax,f/1e6,relativeDB,'Color',blue,'LineWidth',.75);
style(ax,'Frequency (MHz)','Power (dBc/bin)',sprintf('SNR %.4f dB; THD %.4f dB; ENOB %.4f bit',snr,thd,enob));
xlim(ax,[0 cfg.analysis_bandwidth_hz/1e6]); ylim(ax,[-130 5]);
ax=nexttile(tl); count=min(M,ceil(3*fs/(k*fs/M))); t=(0:count-1)'/fs*1e6;
yyaxis(ax,'left'); plot(ax,t,D.vin_diff_v(1:count),'Color',blue,'LineWidth',1.1); ylabel(ax,'Differential input (V)'); ylim(ax,[-1.1 1.1]);
yyaxis(ax,'right'); stairs(ax,t,D.code(1:count),'Color',orange,'LineWidth',1); ylabel(ax,'Output code'); ylim(ax,[-2 65]);
xlabel(ax,'Time (\mus)'); title(ax,'ADC sampled input and output code'); grid(ax,'on'); set(ax,'FontName','Times New Roman','FontSize',12,'Box','on');
legend(ax,{'Input','Code'},'Location','northeast','Box','off');
savePlot(fig,outDir,'04_dynamic_spectrum_waveform');

metrics.passed = m1.average_lsb_v>=.028125 && m1.average_lsb_v<=.034375 && ...
    abs(m2.v1_v)>.9*vref && abs(m2.v63_v)>.9*vref && m2.saturation_verified && ...
    metrics.static.missing_valid_codes==0 && max(abs(dnl),[],'omitnan')<=.5 && max(abs(inl),[],'omitnan')<=.5 && ...
    snr>=36.5 && snr<=37.6 && thd>=-60 && thd<=-50 && enob>=5.8 && enob<=6;
fid=fopen(fullfile(outDir,'matlab_metrics.json'),'w','n','UTF-8');
fprintf(fid,'%s',jsonencode(metrics,'PrettyPrint',true)); fclose(fid);
fprintf('%s\n',jsonencode(metrics)); fprintf('S28_MATLAB_POSTPROCESS_COMPLETE\n');
end

function T=readSamples(path,fs)
T=readtable(path,'VariableNamingRule','preserve');
assert(all(T.code==T.bus_code) && all(T.code>=0 & T.code<=63),'Code/bus mismatch');
assert(all(abs(diff(T.sample_time_s)-1/fs)<1e-12),'Sample interval mismatch');
assert(all(isfinite(T.vin_diff_v)),'Invalid measured input');
end

function [m,G]=dcMetrics(T,lsb)
start=[1; find(abs(diff(T.vin_diff_v))>1e-12)+1]; stop=[start(2:end)-1;height(T)];
v=T.vin_diff_v(start); c=zeros(size(v));
for j=1:numel(start)
    ix=max(start(j),stop(j)-3):stop(j); c(j)=median(T.code(ix));
end
G=table(v,c,'VariableNames',{'vin_diff_v','code'});
lo1=max(v(c<1)); hi1=min(v(c>=1)); lo63=max(v(c<63)); hi63=min(v(c>=63));
assert(lo1<hi1 && lo63<hi63,'Invalid transition brackets');
v1=(lo1+hi1)/2; v63=(lo63+hi63)/2;
m=struct('v1_v',v1,'v63_v',v63,'v1_bracket_v',[lo1 hi1],...
    'v63_bracket_v',[lo63 hi63],'v1_resolution_lsb',(hi1-lo1)/lsb,...
    'v63_resolution_lsb',(hi63-lo63)/lsb,'average_lsb_v',(v63-v1)/62,...
    'lsb_bound_v',[(lo63-hi1)/62 (hi63-lo1)/62],...
    'codes_seen',numel(unique(T.code)),'saturation_verified',all(c(v<-1)==0)&&all(c(v>1)==63));
end

function fig=makeFigure
fig=figure('Visible','off','Color','white','Position',[100 100 1100 780]);
end
function style(ax,x,y,t)
xlabel(ax,x);ylabel(ax,y);title(ax,t);grid(ax,'on');
set(ax,'FontName','Times New Roman','FontSize',12,'Box','on','LineWidth',.75);
end
function savePlot(fig,outDir,name)
drawnow;
exportgraphics(fig,fullfile(outDir,[name '.png']),'Resolution',300,'BackgroundColor','white');
close(fig);
end
