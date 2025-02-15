"use strict";
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
var __generator = (this && this.__generator) || function (thisArg, body) {
    var _ = { label: 0, sent: function() { if (t[0] & 1) throw t[1]; return t[1]; }, trys: [], ops: [] }, f, y, t, g = Object.create((typeof Iterator === "function" ? Iterator : Object).prototype);
    return g.next = verb(0), g["throw"] = verb(1), g["return"] = verb(2), typeof Symbol === "function" && (g[Symbol.iterator] = function() { return this; }), g;
    function verb(n) { return function (v) { return step([n, v]); }; }
    function step(op) {
        if (f) throw new TypeError("Generator is already executing.");
        while (g && (g = 0, op[0] && (_ = 0)), _) try {
            if (f = 1, y && (t = op[0] & 2 ? y["return"] : op[0] ? y["throw"] || ((t = y["return"]) && t.call(y), 0) : y.next) && !(t = t.call(y, op[1])).done) return t;
            if (y = 0, t) op = [op[0] & 2, t.value];
            switch (op[0]) {
                case 0: case 1: t = op; break;
                case 4: _.label++; return { value: op[1], done: false };
                case 5: _.label++; y = op[1]; op = [0]; continue;
                case 7: op = _.ops.pop(); _.trys.pop(); continue;
                default:
                    if (!(t = _.trys, t = t.length > 0 && t[t.length - 1]) && (op[0] === 6 || op[0] === 2)) { _ = 0; continue; }
                    if (op[0] === 3 && (!t || (op[1] > t[0] && op[1] < t[3]))) { _.label = op[1]; break; }
                    if (op[0] === 6 && _.label < t[1]) { _.label = t[1]; t = op; break; }
                    if (t && _.label < t[2]) { _.label = t[2]; _.ops.push(op); break; }
                    if (t[2]) _.ops.pop();
                    _.trys.pop(); continue;
            }
            op = body.call(thisArg, _);
        } catch (e) { op = [6, e]; y = 0; } finally { f = t = 0; }
        if (op[0] & 5) throw op[1]; return { value: op[0] ? op[1] : void 0, done: true };
    }
};
Object.defineProperty(exports, "__esModule", { value: true });
var inquirer_1 = require("inquirer");
var child_process_1 = require("child_process");
function selectProfile() {
    return __awaiter(this, void 0, void 0, function () {
        var profiles, selectedProfile, _a, email, username;
        return __generator(this, function (_b) {
            switch (_b.label) {
                case 0:
                    profiles = [
                        'Erica Trans',
                        'Sean Gayzin',
                        'Ohm Guptra',
                        'Gayden Shun'
                    ];
                    return [4 /*yield*/, inquirer_1.default.prompt([
                            {
                                type: 'list',
                                name: 'selectedProfile',
                                message: 'Please select a profile:',
                                choices: profiles
                            }
                        ])];
                case 1:
                    selectedProfile = (_b.sent()).selectedProfile;
                    console.log("You selected: ".concat(selectedProfile));
                    return [4 /*yield*/, inquirer_1.default.prompt([
                            {
                                type: 'input',
                                name: 'email',
                                message: 'Enter the new email for GitHub:',
                                validate: function (input) { return input.includes('@') ? true : 'Please enter a valid email address'; }
                            },
                            {
                                type: 'input',
                                name: 'username',
                                message: 'Enter the new username for GitHub:'
                            }
                        ])];
                case 2:
                    _a = _b.sent(), email = _a.email, username = _a.username;
                    updateGitHubConfig(email, username);
                    return [2 /*return*/];
            }
        });
    });
}
function updateGitHubConfig(email, username) {
    (0, child_process_1.exec)("git config --global user.email \"".concat(email, "\""), function (error, stdout, stderr) {
        if (error) {
            console.error("Error updating email: ".concat(error.message));
            return;
        }
        if (stderr) {
            console.error("Error: ".concat(stderr));
            return;
        }
        console.log("GitHub email updated to ".concat(email));
    });
    (0, child_process_1.exec)("git config --global user.name \"".concat(username, "\""), function (error, stdout, stderr) {
        if (error) {
            console.error("Error updating username: ".concat(error.message));
            return;
        }
        if (stderr) {
            console.error("Error: ".concat(stderr));
            return;
        }
        console.log("GitHub username updated to ".concat(username));
    });
}
selectProfile();
