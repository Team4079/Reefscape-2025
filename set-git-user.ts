import inquirer from 'inquirer';
import { exec } from 'child_process';

interface ErrorType {
    message: string;
}

interface ProfileType {
    name: string;
    email: string;
}

async function selectProfile(): Promise<void> {
    const userObjects: ProfileType[] = [
        {
            "name": "Erica Trans",
            "email": "thelittlebotengineer@gmail.com"
        },
        {
            "name": "Sean Gayzin",
            "email": "shawngazin@protonmail.com"
        },
        {
            "name": "Ohm Gupta",
            "email": "om@malefic.xyz"
        },
        {
            "name": "Gayden Shun",
            "email": "thelittlebotengineer@gmail.com"
        }
    ]

    const { selectedProfile } = await inquirer.prompt([
        {
            type: 'list',
            name: 'selectedProfile',
            message: 'Please select a profile:',
            choices: userObjects.map((userObject: ProfileType): string => userObject.name)
        }
    ]);

    console.log(`You selected: ${selectedProfile}`);

    const selectedUser: ProfileType = userObjects.find((userObject: ProfileType): boolean => userObject.name === selectedProfile) as ProfileType;
    const selectedEmail: string = selectedUser.email;

    updateGitHubConfig(selectedEmail, selectedProfile);
}

function updateGitHubConfig(email: string, username: string): void {
    exec(`git config --global user.email "${email}"`, (error: ErrorType | null, stdout: string, stderr: string): void => {
        if (error) {
            // console.error(`Error updating email: ${error.message}`);
            return;
        }
        if (stderr) {
            console.error(`Error: ${stderr}`);
            return;
        }
        // console.log(`GitHub email updated to ${email}`);
    });

    exec(`git config --global user.name "${username}"`, (error: ErrorType | null, stdout: string, stderr: string): void => {
        if (error) {
            console.error(`Error updating username: ${error.message as string}`);
            return;
        }
        if (stderr) {
            console.error(`Error: ${stderr}`);
            return;
        }
        console.log(`GitHub username updated to ${username}`);
    });
}

selectProfile().then(r => console.log('Profile selected'));