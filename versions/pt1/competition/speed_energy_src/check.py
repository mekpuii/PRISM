import formulas, sys, time, pickle
t=time.time()
xl = formulas.ExcelModel().loads(sys.argv[1]).finish()
sol = xl.calculate()
print("calc s", round(time.time()-t))
out={}
for k,v in sol.items():
    try: a=v.value
    except Exception: continue
    out[k]=a.tolist() if hasattr(a,'tolist') else a
pickle.dump(out, open(sys.argv[2],"wb")); print(len(out))
