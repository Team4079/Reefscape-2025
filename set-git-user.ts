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

    const { email, username } = await inquirer.prompt([
        {
            type: 'input',
            name: 'email',
            message: 'Enter the new email for GitHub:',
            validate: (input: string): true | string => input.includes('@') ? true : 'Please enter a valid email address'
        },
        {
            type: 'input',
            name: 'username',
            message: 'Enter the new username for GitHub:'
        }
    ]);

    updateGitHubConfig(email, username);
}

function updateGitHubConfig(email: string, username: string): void {
    exec(`git config --global user.email "${email}"`, (error: ErrorType | null, stdout: string, stderr: string): void => {
        if (error) {
            console.error(`Error updating email: ${error.message}`);
            return;
        }
        if (stderr) {
            console.error(`Error: ${stderr}`);
            return;
        }
        console.log(`GitHub email updated to ${email}`);
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

selectProfile()