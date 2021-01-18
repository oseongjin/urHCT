import random

def calVal (getVal):
    epsiVal = 0.00000000000003
    if getVal < 27:
        chkStr = "low"
        rtnVal = 27
    elif 27 <= getVal <= 35:
        rtnVal = (epsiVal * getVal**2 - getVal + 35.5) + getVal + random.random()
        chkStr = "ok"
    elif getVal > 35:
        chkStr = "high"
        rtnVal = 35
    return chkStr, round(rtnVal, 2)

print(calVal(26))
print(calVal(27))
print(calVal(28.12))
print(calVal(29.14))
print(calVal(30))
print(calVal(31))
