using UnityEngine;

public class Mathf {
    public const float Deg2Rad = UnityEngine.Mathf.Deg2Rad;
    public const float Epsilon = UnityEngine.Mathf.Epsilon;
    public const float Infinity = UnityEngine.Mathf.Infinity;
    public const float NegativeInfinity = UnityEngine.Mathf.NegativeInfinity;
    public const float Rad2Deg = UnityEngine.Mathf.Rad2Deg;
    public const float PI = UnityEngine.Mathf.PI;
    public const float TAU = UnityEngine.Mathf.PI*2;

    public static int Abs(int a) { return UnityEngine.Mathf.Abs(a); }
    public static float Abs(float a) { return UnityEngine.Mathf.Abs(a); }
    public static float Acos(float a) { return UnityEngine.Mathf.Acos(a); }
    public static bool Approximately(float a,float b) { return UnityEngine.Mathf.Approximately(a,b); }
    public static float Asin(float a) { return UnityEngine.Mathf.Asin(a); }
    public static float Atan(float a) { return UnityEngine.Mathf.Atan(a); }
    public static float Atan2(float y,float x) { return UnityEngine.Mathf.Atan2(y,x); }
    public static float Ceil(float a) { return UnityEngine.Mathf.Ceil(a); }
    public static int CeilToInt(float a) { return UnityEngine.Mathf.CeilToInt(a); }
    public static int Clamp(int value,int min,int max) { return UnityEngine.Mathf.Clamp(value,min,max); }
    public static float Clamp(float value,float min,float max) { return UnityEngine.Mathf.Clamp(value,min,max); }
    public static float Clamp01(float value) { return UnityEngine.Mathf.Clamp01(value); }
    public static int ClosestPowerOfTwo(int a) { return UnityEngine.Mathf.ClosestPowerOfTwo(a); }
    public static float Cos(float a) { return UnityEngine.Mathf.Cos(a); }
    public static float DeltaAngle(float current,float target) { return UnityEngine.Mathf.DeltaAngle(current,target); }
    public static float Exp(float power) { return UnityEngine.Mathf.Exp(power); }
    public static float Floor(float a) { return UnityEngine.Mathf.Floor(a); }
    public static int FloorToInt(float a) { return UnityEngine.Mathf.FloorToInt(a); }
    public static float Gamma(float value,float absmax,float gamma) { return UnityEngine.Mathf.Gamma(value,absmax,gamma); }
    public static float GammaToLinearSpace(float value) { return UnityEngine.Mathf.GammaToLinearSpace(value); }
    public static float InverseLerp(float from,float to,float value) { return UnityEngine.Mathf.InverseLerp(from,to,value); }
    public static bool IsPowerOfTwo(int a) { return UnityEngine.Mathf.IsPowerOfTwo(a); }
    public static float Lerp(float from,float to,float t) { return UnityEngine.Mathf.Lerp(from,to,t); }
    public static float LerpAngle(float a,float b,float t) { return UnityEngine.Mathf.LerpAngle(a,b,t); }
    public static float LinearToGammaSpace(float value) { return UnityEngine.Mathf.LinearToGammaSpace(value); }
    public static float Log(float value) { return UnityEngine.Mathf.Log(value); }
    public static float Log10(float value) { return UnityEngine.Mathf.Log10(value); }
    public static int Max(int a,int b) { return UnityEngine.Mathf.Max(a,b); }
    public static float Max(float a,float b) { return UnityEngine.Mathf.Max(a,b); }
    public static int Max(params int[] values) { return UnityEngine.Mathf.Max(values); }
    public static float Max(params float[] values) { return UnityEngine.Mathf.Max(values); }
    public static int Min(int a,int b) { return UnityEngine.Mathf.Min(a,b); }
    public static float Min(float a,float b) { return UnityEngine.Mathf.Min(a,b); }
    public static int Min(params int[] values) { return UnityEngine.Mathf.Min(values); }
    public static float Min(params float[] values) { return UnityEngine.Mathf.Min(values); }
    public static float MoveTowards(float current,float target,float maxDelta) { return UnityEngine.Mathf.MoveTowards(current,target,maxDelta); }
    public static float MoveTowardsAngle(float current,float target,float maxDelta) { return UnityEngine.Mathf.MoveTowardsAngle(current,target,maxDelta); }
    public static int NextPowerOfTwo(int a) { return UnityEngine.Mathf.NextPowerOfTwo(a); }
    public static float PerlinNoise(float x,float y) { return UnityEngine.Mathf.PerlinNoise(x,y); }
    public static float PingPong(float t,float length) { return UnityEngine.Mathf.PingPong(t,length); }
    public static float Pow(float f,float p) { return UnityEngine.Mathf.Pow(f,p); }
    public static float Repeat(float t,float length) { return Mathf.Repeat(t,length); }
    public static float Round(float f) { return UnityEngine.Mathf.Round(f); }
    public static int RoundToInt(float f) { return UnityEngine.Mathf.RoundToInt(f); }
    public static float Sign(float f) { return UnityEngine.Mathf.Sign(f); }
    public static float Sin(float f) { return UnityEngine.Mathf.Sin(f); }
    public static float SmoothDamp(float current,float target,ref float currentVelocity,float smoothTime) { return UnityEngine.Mathf.SmoothDamp(current,target,ref currentVelocity,smoothTime); }
    public static float SmoothDamp(float current,float target,ref float currentVelocity,float smoothTime,float maxSpeed) { return UnityEngine.Mathf.SmoothDamp(current,target,ref currentVelocity,smoothTime,maxSpeed); }
    public static float SmoothDamp(float current,float target,ref float currentVelocity,float smoothTime,float maxSpeed,float deltaTime) { return UnityEngine.Mathf.SmoothDamp(current,target,ref currentVelocity,smoothTime,maxSpeed,deltaTime); }
    public static float SmoothDampAngle(float current,float target,ref float currentVelocity,float smoothTime) { return UnityEngine.Mathf.SmoothDampAngle(current,target,ref currentVelocity,smoothTime); }
    public static float SmoothDampAngle(float current,float target,ref float currentVelocity,float smoothTime,float maxSpeed) { return UnityEngine.Mathf.SmoothDampAngle(current,target,ref currentVelocity,smoothTime,maxSpeed); }
    public static float SmoothDampAngle(float current,float target,ref float currentVelocity,float smoothTime,float maxSpeed,float deltaTime) { return UnityEngine.Mathf.SmoothDampAngle(current,target,ref currentVelocity,smoothTime,maxSpeed,deltaTime); }
    public static float SmoothStep(float from,float to,float t) { return UnityEngine.Mathf.SmoothStep(from,to,t); }
    public static float Sqrt(float f) { return UnityEngine.Mathf.Sqrt(f); }
    public static float Tan(float f) { return UnityEngine.Mathf.Tan(f); }
}