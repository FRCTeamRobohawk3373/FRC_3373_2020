 package frc.team3373;

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */


import edu.wpi.first.wpilibj.Joystick;
/*
 *
 * @author Jamie Dyer
 * @author Drew Marino (not really)
 * 
 */
public class SuperJoystick extends Joystick{
         
   private boolean flagA;
   boolean flagB;
   boolean flagX;
   boolean flagY;
   boolean flagLB;
   boolean flagRB;
   boolean flagStart;
   boolean flagBack;
   boolean flagLStick;
   boolean flagRStick;
   boolean flagDPadUp;
   boolean flagDPadDown;
   boolean flagDPadLeft;
   boolean flagDPadRight;
   boolean flagDPadUpRight;
   boolean flagDPadUpLeft;
   boolean flagDPadDownRight;
   boolean flagDPadDownLeft;
   boolean flagDPadNotPushed;
    SuperJoystick(int port){
        super(port); //also need to clear joystick class
        clearButtons();
    }
    /********************************
     * Checks if a button is pushed 
     * @param button the raw button number
     * @return whether a button is pushed or not (true = pushed)
     * @see Joystick
     ********************************/
    public boolean isButtonPushed(int button){
        return getRawButton(button);
    }
    public boolean isAPushed(){
        if(getRawButton(1) && flagA){
            flagA = false;
            return true;
        }
        else{
            return false;
        }
    }    
    public boolean isBPushed(){
        if(getRawButton(2) && flagB){
            flagB = false;
            return true;
        }
        else{
            return false;
        }
    }
    public boolean isXPushed(){
        if(getRawButton(3) && flagX){
            flagX = false;
            return true;
        }
        else{
            return false;
        }
    }
    public boolean isYPushed(){
        if(getRawButton(4) && flagY){
            flagY = false;
            return true;
        }
        else{
            return false;
        }
    }
    public boolean isLBPushed(){
        if(getRawButton(5) && flagLB){
            flagLB = false;
            return true;
        }
        else{
            return false;
        }
    }
    public boolean isRBPushed(){
        if(getRawButton(6) && flagRB){
            flagRB = false;
            return true;
        }
        else{
            return false;
        }
    }
    public boolean isBackPushed(){
        if(getRawButton(7) && flagBack){
            flagBack = false;
            return true;
        }
        else{
            return false;
        }
    }
    public boolean isStartPushed(){
        if(getRawButton(8) && flagStart){
            flagStart = false;
            return true;
        }
        else{
            return false;
        }
    }
    public boolean isLStickPushed(){
        if(getRawButton(9) && flagLStick){
            flagLStick = false;
            return true;
        }
        else{
            return false;
        }
    }
    public boolean isRStickPushed(){
        if(getRawButton(10) && flagRStick){
            flagRStick = false;
            return true;
        }
        else{
            return false;
        }
    }
    public boolean isDPadUpPushed(){
    	if(getPOV() == 0 && flagDPadUp){
    		flagDPadUp = false;
    		return true;
    	} else {
    		return false;
    	}
    }
    public boolean isDPadDownPushed(){
    	if(getPOV() == 180 && flagDPadDown){
    		flagDPadDown = false;
    		return true;
    	} else {
    		return false;
    	}
    }
    public boolean isDPadLeftPushed(){
    	if(getPOV() == 270 && flagDPadLeft){
    		flagDPadLeft = false;
    		return true;
    	} else {
    		return false;
    	}
    }
    public boolean isDPadRightPushed(){
    	if(getPOV() == 90 && flagDPadRight){
    		flagDPadRight = false;
    		return true;
    	} else {
    		return false;
    	}
    }
    public boolean isDPadUpRightPushed(){
    	if(getPOV() == 45 && flagDPadUpRight){
    		flagDPadUpRight = false;
    		return true;
    	} else {
    		return false;
    	}
    }
    public boolean isDPadUpLeftPushed(){
    	if(getPOV() == 315 && flagDPadUpLeft){
    		flagDPadUpLeft = false;
    		return true;
    	} else {
    		return false;
    	}
    }
    public boolean isDPadDownRightPushed(){
    	if(getPOV() == 135 && flagDPadDownRight){
    		flagDPadDownRight = false;
    		return true;
    	} else {
    		return false;
    	}
    }
    public boolean isDPadDownLeftPushed(){
    	if(getPOV() == 225 && flagDPadDownLeft){
    		flagDPadDownLeft = false;
    		return true;
    	} else {
    		return false;
    	}
    }
    public boolean isDPadNotPushed(){
    	if(getPOV() == -1 && flagDPadNotPushed){
    		flagDPadNotPushed = false;
    		return true;
    	} else {
    		return false;
    	}
    }
    public boolean isAHeld(){
        return getRawButton(1);
    }
    public boolean isBHeld(){
        return getRawButton(2);
    }
    public boolean isXHeld(){
        return getRawButton(3);
    }
    public boolean isYHeld(){
        return getRawButton(4);
    }
    public boolean isLBHeld(){
        return getRawButton(5);
    }
    public boolean isRBHeld(){
        return getRawButton(6);
    }
    public boolean isBackHeld(){
        return getRawButton(7);
    }
    public boolean isStartHeld(){
        return getRawButton(8);
    }
    public boolean isLStickHeld(){
        return getRawButton(9);
    }
    public boolean isRStickHeld(){
        return getRawButton(10);
    }
    public boolean isDPadUpHeld(){
    	return (getPOV() == 0);
    }
    public boolean isDPadDownHeld(){
    	return (getPOV() == 180);
    }
    public boolean isDPadLeftHeld(){
    	return (getPOV() == 270);
    }
    public boolean isDPadRightHeld(){
    	return (getPOV() == 90);
    }
    public boolean isDPadUpRightHeld(){
    	return (getPOV() == 45);
    }
    public boolean isDPadUpLeftHeld(){
    	return (getPOV() == 315);
    }
    public boolean isDPadDownRightHeld(){
    	return (getPOV() == 135);
    }
    public boolean isDPadDownLeftHeld(){
    	return (getPOV() == 225);
    }
    public boolean isDPadNotHeld(){
    	return (getPOV() == -1);
    }
    public void clearButtons(){
        if (!flagA && !getRawButton(1)) { //toggles
            flagA = true;
        } if (!flagB && !getRawButton(2)){
            flagB = true;
        } if (!flagX && !getRawButton(3)){
            flagX = true;
        } if (!flagY && !getRawButton(4)){
            flagY = true;
        } if (!flagLB && !getRawButton(5)){
            flagLB = true;
        } if (!flagRB && !getRawButton(6)){
            flagRB = true;
        } if (!flagBack && !getRawButton(7)){
            flagBack = true;
        } if (!flagStart && !getRawButton(8)){
            flagStart = true;
        } if (!flagLStick && !getRawButton(9)){
            flagLStick = true;
        } if (!flagRStick && !getRawButton(10)){
            flagRStick = true;
        }
    }
    public void clearB(){
     if (!flagB && !getRawButton(2)){
        flagB = true;
    }
    }
     public void clearA(){
         if (!flagA && !getRawButton(1)){
            flagA = true;
        }
}
     public void clearX(){
    	 if (!flagX && !getRawButton(3)){
             flagX = true;
     }
     }
     public void clearY(){
    	 if (!flagY && !getRawButton(4)){
             flagY = true;
         }
     }
     public void clearLB(){
    	 if (!flagLB && !getRawButton(5)){
             flagLB = true;
         }
     }
     public void clearRB(){
    	 if (!flagRB && !getRawButton(6)){
             flagRB = true;
     }
     }
     public void clearBack(){
    	 if (!flagBack && !getRawButton(7)){
             flagBack = true;
         }
     }
     public void clearStart(){
    	 if (!flagStart && !getRawButton(8)){
             flagStart = true;
     }
     }
     public void clearLStick(){
    	 if (!flagLStick && !getRawButton(9)){
             flagLStick = true;
    	 }
     }
     public void clearRStick(){
    	 if (!flagRStick && !getRawButton(10)){
             flagRStick = true;
    	 }
     }
     public void clearDPad(){
    	 if(!flagDPadUp && getPOV() != 0){
    		 flagDPadUp = true;
    	 }
    	 if(!flagDPadDown && getPOV() != 180){
    		 flagDPadDown = true;
    	 }
    	 if(!flagDPadLeft && getPOV() != 270){
    		 flagDPadLeft = true;
    	 }
    	 if(!flagDPadRight && getPOV() != 90){
    		 flagDPadRight = true;
    	 }
    	 if(!flagDPadNotPushed && getPOV() != -1){
    		 flagDPadNotPushed = true;
    	 }
    	 if(!flagDPadUpRight && getPOV() != 45){
    		 flagDPadUpRight = true;
    	 }
    	 if(!flagDPadUpLeft && getPOV() != 315){
    		 flagDPadUpLeft = true;
    	 }
    	 if(!flagDPadDownRight && getPOV() != 135){
    		 flagDPadDownRight = true;
    	 }
    	 if(!flagDPadDownLeft && getPOV() != 225){
    		 flagDPadDownLeft = true;
    	 }
     }
     public void clearDPadUp(){
    	 if(!flagDPadUp && getPOV() != 0){
    		 flagDPadUp = true;
    	 }
     }
     public void clearDPadDown(){
    	 if(!flagDPadDown && getPOV() != 180){
    		 flagDPadDown = true;
    	 }
     }
     public void clearDPadLeft(){
    	 if(!flagDPadLeft && getPOV() != 270){
    		 flagDPadLeft = true;
    	 }
     }
     public void clearDPadRight(){
    	 if(!flagDPadRight && getPOV() != 0){
    		 flagDPadRight = true;
    	 }
     }
     public void clearDPadNotPushed(){
    	 if(!flagDPadNotPushed && getPOV() != -1){
    		 flagDPadNotPushed = true;
    	 }
     }
     public void clearDPadUpRight(){
    	 if(!flagDPadUpRight && getPOV() != 45){
    		 flagDPadUpRight = true;
    	 }
     }
     public void clearDPadUpLeft(){
    	 if(!flagDPadUpLeft && getPOV() != 315){
    		 flagDPadUpLeft = true;
    	 } 
     }
     public void clearDPadDownRight(){
    	 if(!flagDPadDownRight && getPOV() != 135){
    		 flagDPadDownRight = true;
    	 }
     }
     public void clearDPadDownLeft(){
    	 if(!flagDPadDownLeft && getPOV() != 225){
    		 flagDPadDownLeft = true;
    	 }
     }
}