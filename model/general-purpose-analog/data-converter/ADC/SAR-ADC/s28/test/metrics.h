#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>

namespace s28_test {
constexpr double pi = 3.1415926535897932384626433832795;
inline double db(double a, double b) {
    if (!(a > 0) || !(b > 0)) return std::numeric_limits<double>::quiet_NaN();
    return 10*std::log10(a/b);
}
inline void number(std::ostream& o, double v) { if (std::isfinite(v)) o << std::setprecision(17) << v; else o << "null"; }
inline void fft(std::vector<std::complex<double>>& a) {
    const auto n = a.size();
    if (n < 2 || (n & (n-1))) throw std::runtime_error("FFT size must be a power of two");
    for (std::size_t i=1,j=0; i<n; ++i) {
        auto bit=n>>1;
        for (; j&bit; bit>>=1) j ^= bit;
        j ^= bit; if (i<j) std::swap(a[i],a[j]);
    }
    for (std::size_t len=2; len<=n; len<<=1) {
        auto wlen=std::polar(1.0,-2*pi/static_cast<double>(len));
        for (std::size_t i=0; i<n; i+=len) {
            std::complex<double> w=1;
            for (std::size_t j=0; j<len/2; ++j) {
                auto u=a[i+j], v=a[i+j+len/2]*w;
                a[i+j]=u+v; a[i+j+len/2]=u-v; w *= wlen;
            }
        }
    }
}
struct Spectrum {
    std::vector<double> power;
    std::vector<unsigned> harmonic_bins;
    double snr=0, thd=0, sndr=0, sfdr=0, enob=0, spur_hz=0;
    double signal=0, noise=0, distortion=0, total_ac=0;
    unsigned spur_bin=0;
};
inline Spectrum analyze(const std::vector<double>& y,double fs,unsigned k,unsigned order,double bandwidth) {
    const unsigned n=static_cast<unsigned>(y.size());
    if (n<256 || k==0 || k>=n/2 || order<2 || bandwidth<=0 || bandwidth>fs/2)
        throw std::runtime_error("Invalid spectral configuration");
    double mean=0; for (auto v:y) mean+=v; mean/=n;
    std::vector<std::complex<double>> a(n);
    for (unsigned i=0;i<n;++i) a[i]=y[i]-mean;
    fft(a); Spectrum s; s.power.resize(n/2+1);
    for (unsigned i=0;i<=n/2;++i) s.power[i]=std::norm(a[i])/(static_cast<double>(n)*n)*(i>0&&i<n/2?2:1);
    const unsigned end=static_cast<unsigned>(std::floor(bandwidth/fs*n+1e-9));
    if (k>end) throw std::runtime_error("Fundamental outside analysis band");
    std::set<unsigned> bins;
    for (unsigned h=2;h<=order;++h) {
        const unsigned raw=(h*k)%n, bin=std::min(raw,n-raw);
        s.harmonic_bins.push_back(bin);
        if (bin && bin!=k && bin<=end) bins.insert(bin);
    }
    s.signal=s.power[k]; double maxspur=0;
    for (unsigned i=1;i<=end;++i) {
        s.total_ac+=s.power[i]; if (i==k) continue;
        if (bins.count(i)) s.distortion+=s.power[i]; else s.noise+=s.power[i];
        if (s.power[i]>maxspur) { maxspur=s.power[i]; s.spur_bin=i; }
    }
    s.snr=db(s.signal,s.noise); s.thd=db(s.distortion,s.signal);
    s.sndr=db(s.signal,s.noise+s.distortion); s.sfdr=db(s.signal,maxspur);
    s.enob=(s.sndr-1.76)/6.02; s.spur_hz=s.spur_bin*fs/n; return s;
}
inline void selftest() {
    const unsigned n=4096,k=37;
    std::vector<double> y(n);
    for (unsigned i=0;i<n;++i) y[i]=std::sin(2*pi*k*i/n)+.01*std::sin(2*pi*2*k*i/n);
    const auto s=analyze(y,100e6,k,10,50e6);
    if (std::abs(s.thd+40)>1e-8 || std::abs(s.sfdr-40)>1e-8 || std::abs(s.total_ac-.50005)>1e-10)
        throw std::runtime_error("FFT normalization/harmonic self-test failed");
    // Fold a second harmonic above Nyquist to its unique in-band bin.
    for (unsigned i=0;i<n;++i) y[i]=std::sin(2*pi*1501*i/n)+.01*std::sin(2*pi*3002*i/n);
    const auto folded=analyze(y,100e6,1501,2,50e6);
    if (folded.harmonic_bins[0]!=1094 || std::abs(folded.thd+40)>1e-8)
        throw std::runtime_error("Harmonic folding self-test failed");
}
} // namespace s28_test
