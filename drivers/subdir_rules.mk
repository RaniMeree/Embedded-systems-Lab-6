################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
drivers/%.obj: ../drivers/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/Users/ranimeree/Desktop/MDU/Embedded systems I DVA454 24113 HT2025/ti-cgt-arm_18.1.4.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/Users/ranimeree/Desktop/MDU/Embedded systems I DVA454 24113 HT2025/Workspace/LAB-2" --include_path="C:/TivaWare_C_Series-2.1.4.178/grlib" --include_path="C:/Users/ranimeree/Desktop/MDU/Embedded systems I DVA454 24113 HT2025/TivaWare_C_Series-2.1.4.178" --include_path="C:/Users/ranimeree/Desktop/MDU/Embedded systems I DVA454 24113 HT2025/ti-cgt-arm_18.1.4.LTS/include" --define=ccs="ccs" --define=PART_TM4C129ENCPDT -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="drivers/$(basename $(<F)).d_raw" --obj_directory="drivers" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


