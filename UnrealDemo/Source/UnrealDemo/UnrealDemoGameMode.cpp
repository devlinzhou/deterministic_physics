// Copyright Epic Games, Inc. All Rights Reserved.

#include "UnrealDemoGameMode.h"
#include "UnrealDemoCharacter.h"
#include "UObject/ConstructorHelpers.h"

AUnrealDemoGameMode::AUnrealDemoGameMode()
{
	// set default pawn class to our Blueprinted character
	static ConstructorHelpers::FClassFinder<APawn> PlayerPawnBPClass(TEXT("/Game/ThirdPersonCPP/Blueprints/ThirdPersonCharacter"));
	if (PlayerPawnBPClass.Class != NULL)
	{
		DefaultPawnClass = PlayerPawnBPClass.Class;
	}
}
